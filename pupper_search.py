#!/usr/bin/env python3
"""Semantic search over Mini Pupper documentation and Discord logs using DuckDB + OpenRouter embeddings."""
from __future__ import annotations

import hashlib
import json
import os
import re
import sys
import time
from pathlib import Path
from textwrap import dedent
from typing import Optional

import duckdb
import httpx

# --- Config ---
DB_PATH = Path(__file__).parent / "pupper.duckdb"
EMBEDDING_MODEL = "openai/text-embedding-3-small"
EMBEDDING_DIMS = 1536
OPENROUTER_KEY = os.environ.get("OPENROUTER_API_KEY", "")
CHUNK_SIZE = 800  # chars
CHUNK_OVERLAP = 200
BATCH_SIZE = 50  # embeddings per API call


def get_api_key() -> str:
    key = OPENROUTER_KEY
    if not key:
        # Try reading from an explicit per-user key file. Project files should
        # never contain API keys.
        for p in [Path.home() / ".config" / "openrouter" / "api_key"]:
            if p.exists():
                for line in p.read_text().splitlines():
                    if "OPENROUTER_API_KEY" in line:
                        key = line.split("=", 1)[-1].strip().strip('"').strip("'")
                        if key:
                            return key
    if not key:
        raise RuntimeError("Set OPENROUTER_API_KEY environment variable")
    return key


# --- Embedding ---
def embed_texts(texts: list[str], api_key: str) -> list[list[float]]:
    """Get embeddings from OpenRouter."""
    resp = httpx.post(
        "https://openrouter.ai/api/v1/embeddings",
        headers={"Authorization": f"Bearer {api_key}"},
        json={"model": EMBEDDING_MODEL, "input": texts},
        timeout=60,
    )
    resp.raise_for_status()
    data = resp.json()
    # Sort by index to ensure order
    sorted_data = sorted(data["data"], key=lambda x: x["index"])
    return [d["embedding"] for d in sorted_data]


# --- Chunking ---
def chunk_text(text: str, source: str, doc_type: str) -> list[dict]:
    """Split text into overlapping chunks with metadata."""
    chunks = []
    # Split by sections first (headers or blank-line-separated blocks)
    lines = text.split("\n")
    current_section = ""
    current_chunk = ""
    
    for line in lines:
        # Track section headers
        if line.startswith("===") or line.startswith("# ") or line.startswith("## "):
            current_section = line.strip("=#- \n")[:100]
        
        current_chunk += line + "\n"
        
        if len(current_chunk) >= CHUNK_SIZE:
            chunks.append({
                "text": current_chunk.strip(),
                "source": source,
                "section": current_section,
                "doc_type": doc_type,
            })
            # Keep overlap
            overlap_lines = current_chunk.split("\n")
            overlap_text = ""
            for ol in reversed(overlap_lines):
                if len(overlap_text) + len(ol) < CHUNK_OVERLAP:
                    overlap_text = ol + "\n" + overlap_text
                else:
                    break
            current_chunk = overlap_text
    
    # Don't forget the last chunk
    if current_chunk.strip():
        chunks.append({
            "text": current_chunk.strip(),
            "source": source,
            "section": current_section,
            "doc_type": doc_type,
        })
    
    return chunks


def parse_discord_export(text: str) -> list[dict]:
    """Parse Discord export into chunks grouped by conversation flow."""
    chunks = []
    current_channel = "unknown"
    current_chunk = ""
    
    for line in text.split("\n"):
        # Detect channel headers (DiscordChatExporter format)
        if line.startswith("===") or line.startswith("---"):
            if "Channel:" in line or "#" in line:
                current_channel = line.strip("=-# \n")[:100]
        
        current_chunk += line + "\n"
        
        if len(current_chunk) >= CHUNK_SIZE:
            chunks.append({
                "text": current_chunk.strip(),
                "source": "discord-export",
                "section": current_channel,
                "doc_type": "discord",
            })
            # Small overlap for conversation continuity
            overlap_lines = current_chunk.split("\n")[-5:]
            current_chunk = "\n".join(overlap_lines) + "\n"
    
    if current_chunk.strip():
        chunks.append({
            "text": current_chunk.strip(),
            "source": "discord-export",
            "section": current_channel,
            "doc_type": "discord",
        })
    
    return chunks


# --- Database ---
def init_db(conn: duckdb.DuckDBPyConnection) -> None:
    """Initialize schema and vector extension."""
    try:
        conn.execute("INSTALL vss;")
    except duckdb.Error:
        pass
    try:
        conn.execute("LOAD vss;")
    except duckdb.Error:
        pass
    
    conn.execute(f"""
        CREATE TABLE IF NOT EXISTS documents (
            id TEXT PRIMARY KEY,
            source TEXT NOT NULL,
            doc_type TEXT NOT NULL,
            file_hash TEXT NOT NULL,
            ingested_at TIMESTAMP DEFAULT current_timestamp
        );
        
        CREATE TABLE IF NOT EXISTS chunks (
            id TEXT PRIMARY KEY,
            doc_id TEXT NOT NULL REFERENCES documents(id),
            chunk_index INTEGER NOT NULL,
            text TEXT NOT NULL,
            section TEXT,
            embedding FLOAT[{EMBEDDING_DIMS}],
            UNIQUE(doc_id, chunk_index)
        );
    """)
    
    # Try to create HNSW index
    try:
        conn.execute(f"""
            CREATE INDEX IF NOT EXISTS idx_chunks_embedding 
            ON chunks USING HNSW(embedding) 
            WITH (metric = 'cosine');
        """)
    except duckdb.Error:
        pass  # Works without index, just slower


def file_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()[:16]


def ingest_file(conn: duckdb.DuckDBPyConnection, path: Path, doc_type: str, api_key: str) -> int:
    """Ingest a file, skip if unchanged."""
    fhash = file_hash(path)
    source = path.name
    
    # Check if already ingested with same hash
    existing = conn.execute(
        "SELECT id FROM documents WHERE source = ? AND file_hash = ?",
        [source, fhash]
    ).fetchone()
    
    if existing:
        print(f"  ⏭  {source} unchanged, skipping")
        return 0
    
    # Remove old version if exists
    old = conn.execute("SELECT id FROM documents WHERE source = ?", [source]).fetchone()
    if old:
        conn.execute("DELETE FROM chunks WHERE doc_id = ?", [old[0]])
        conn.execute("DELETE FROM documents WHERE id = ?", [old[0]])
    
    # Read and chunk
    text = path.read_text(encoding="utf-8", errors="replace")
    
    if doc_type == "discord":
        raw_chunks = parse_discord_export(text)
    else:
        raw_chunks = chunk_text(text, source, doc_type)
    
    if not raw_chunks:
        print(f"  ⚠️  {source} produced no chunks")
        return 0
    
    # Create document record
    import uuid
    doc_id = str(uuid.uuid4())[:8]
    conn.execute(
        "INSERT INTO documents (id, source, doc_type, file_hash) VALUES (?, ?, ?, ?)",
        [doc_id, source, doc_type, fhash]
    )
    
    # Embed in batches
    total_chunks = len(raw_chunks)
    print(f"  📄 {source}: {total_chunks} chunks")
    
    for batch_start in range(0, total_chunks, BATCH_SIZE):
        batch = raw_chunks[batch_start:batch_start + BATCH_SIZE]
        texts = [c["text"] for c in batch]
        
        embeddings = embed_texts(texts, api_key)
        
        for i, (chunk, emb) in enumerate(zip(batch, embeddings)):
            chunk_id = f"{doc_id}_{batch_start + i}"
            conn.execute(
                "INSERT INTO chunks (id, doc_id, chunk_index, text, section, embedding) VALUES (?, ?, ?, ?, ?, ?)",
                [chunk_id, doc_id, batch_start + i, chunk["text"], chunk["section"], emb]
            )
        
        if batch_start + BATCH_SIZE < total_chunks:
            time.sleep(0.5)  # Rate limiting
    
    conn.commit()
    print(f"  ✅ {source}: {total_chunks} chunks embedded")
    return total_chunks


def search(conn: duckdb.DuckDBPyConnection, query: str, api_key: str, 
           limit: int = 5, doc_type: Optional[str] = None) -> list[dict]:
    """Semantic search across all ingested documents."""
    query_emb = embed_texts([query], api_key)[0]
    
    type_filter = ""
    params = [query_emb, limit]
    if doc_type:
        type_filter = "AND d.doc_type = ?"
        params = [query_emb, doc_type, limit]
    
    results = conn.execute(f"""
        SELECT 
            c.text,
            c.section,
            d.source,
            d.doc_type,
            array_cosine_similarity(c.embedding, ?::FLOAT[{EMBEDDING_DIMS}]) as score
        FROM chunks c
        JOIN documents d ON c.doc_id = d.id
        WHERE c.embedding IS NOT NULL {type_filter}
        ORDER BY score DESC
        LIMIT ?
    """, params).fetchall()
    
    return [
        {
            "text": r[0],
            "section": r[1],
            "source": r[2],
            "doc_type": r[3],
            "score": round(r[4], 4) if r[4] else 0,
        }
        for r in results
    ]


def stats(conn: duckdb.DuckDBPyConnection) -> dict:
    docs = conn.execute("SELECT source, doc_type, file_hash FROM documents ORDER BY source").fetchall()
    chunk_count = conn.execute("SELECT COUNT(*) FROM chunks").fetchone()[0]
    return {"documents": [{"source": d[0], "type": d[1], "hash": d[2]} for d in docs], "total_chunks": chunk_count}


# --- CLI ---
def main():
    if len(sys.argv) < 2:
        print(dedent("""
        Usage:
            python pupper_search.py ingest              Ingest/update all source files
            python pupper_search.py search <query>      Semantic search
            python pupper_search.py search --type docs <query>   Search docs only
            python pupper_search.py search --type discord <query> Search Discord only
            python pupper_search.py stats               Show ingestion stats
        """).strip())
        sys.exit(1)
    
    cmd = sys.argv[1]
    
    conn = duckdb.connect(str(DB_PATH))
    init_db(conn)
    
    if cmd == "ingest":
        api_key = get_api_key()
        project_dir = Path(__file__).parent.parent
        
        sources = [
            (project_dir / "minipupper-docs-complete.txt", "docs"),
            (project_dir / "community-chat-export.txt", "discord"),
        ]
        
        total = 0
        for path, dtype in sources:
            if path.exists():
                total += ingest_file(conn, path, dtype, api_key)
            else:
                print(f"  ⚠️  {path.name} not found at {path}")
        
        print(f"\nDone. {total} new chunks embedded.")
        s = stats(conn)
        print(f"Total: {len(s['documents'])} documents, {s['total_chunks']} chunks")
    
    elif cmd == "search":
        api_key = get_api_key()
        doc_type = None
        query_args = sys.argv[2:]
        
        if "--type" in query_args:
            idx = query_args.index("--type")
            doc_type = query_args[idx + 1]
            query_args = query_args[:idx] + query_args[idx + 2:]
        
        limit = 5
        if "--limit" in query_args:
            idx = query_args.index("--limit")
            limit = int(query_args[idx + 1])
            query_args = query_args[:idx] + query_args[idx + 2:]
        
        query = " ".join(query_args)
        if not query:
            print("Usage: python pupper_search.py search <query>")
            sys.exit(1)
        
        results = search(conn, query, api_key, limit=limit, doc_type=doc_type)
        
        if not results:
            print("No results found.")
            sys.exit(0)
        
        for i, r in enumerate(results, 1):
            print(f"\n{'='*60}")
            print(f"Result {i} | Score: {r['score']} | Source: {r['source']} ({r['doc_type']})")
            if r['section']:
                print(f"Section: {r['section']}")
            print(f"{'─'*60}")
            print(r['text'][:500])
            if len(r['text']) > 500:
                print(f"... ({len(r['text'])} chars total)")
    
    elif cmd == "stats":
        s = stats(conn)
        print(f"Documents: {len(s['documents'])}")
        for d in s['documents']:
            print(f"  • {d['source']} ({d['type']}) [hash: {d['hash']}]")
        print(f"Total chunks: {s['total_chunks']}")
    
    else:
        print(f"Unknown command: {cmd}")
        sys.exit(1)
    
    conn.close()


if __name__ == "__main__":
    main()
