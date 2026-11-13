"""Semantic search over robot documentation and community chat exports.

Chunk local text files, embed them through OpenRouter's OpenAI-compatible
embeddings API, store the vectors in a single DuckDB file, and query by cosine
similarity. Built to answer "has anyone else hit this error?" while debugging
a Mini Pupper, but it works on any pile of text.

    minipupper-docs ingest minipupper-docs.txt
    minipupper-docs ingest --type chat discord-export.txt
    minipupper-docs search "ldlidar communication is abnormal"

Needs the `docs` extra: pip install -e ".[docs]"
"""

from __future__ import annotations

import argparse
import hashlib
import os
import sys
import time
import uuid
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover
    import duckdb

DB_PATH = Path(os.environ.get("MINIPUPPER_DOCS_DB", "pupper_docs.duckdb"))
EMBEDDING_MODEL = os.environ.get("MINIPUPPER_EMBEDDING_MODEL", "openai/text-embedding-3-small")
EMBEDDING_DIMS = 1536
EMBEDDINGS_URL = "https://openrouter.ai/api/v1/embeddings"
CHUNK_SIZE = 800  # characters
CHUNK_OVERLAP = 200  # characters carried into the next doc chunk
CHAT_OVERLAP_LINES = 5  # lines carried into the next chat chunk
BATCH_SIZE = 50  # texts per embeddings request
DOC_TYPES = ("docs", "chat")


def get_api_key() -> str:
    key = os.environ.get("OPENROUTER_API_KEY", "")
    if not key:
        raise SystemExit("Set OPENROUTER_API_KEY to use embeddings.")
    return key


def embed_texts(texts: list[str], api_key: str) -> list[list[float]]:
    import httpx

    resp = httpx.post(
        EMBEDDINGS_URL,
        headers={"Authorization": f"Bearer {api_key}"},
        json={"model": EMBEDDING_MODEL, "input": texts},
        timeout=60,
    )
    resp.raise_for_status()
    return [item["embedding"] for item in sorted(resp.json()["data"], key=lambda item: item["index"])]


# --- Chunking (pure) ---


def _tail(text: str, max_chars: int) -> str:
    """The trailing whole lines of `text` that fit in `max_chars`."""
    kept: list[str] = []
    size = 0
    for line in reversed(text.split("\n")):
        if size + len(line) >= max_chars:
            break
        kept.insert(0, line)
        size += len(line) + 1
    return "\n".join(kept) + ("\n" if kept else "")


def chunk_text(text: str, source: str, doc_type: str = "docs", chunk_size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> list[dict]:
    """Split documentation into ~chunk_size pieces, tagging each with the latest heading."""
    chunks: list[dict] = []
    section = ""
    current = ""
    for line in text.split("\n"):
        if line.startswith(("===", "# ", "## ")):
            heading = line.strip("=#- \n")[:100]
            if heading:
                section = heading
        current += line + "\n"
        if len(current) >= chunk_size:
            chunks.append({"text": current.strip(), "source": source, "section": section, "doc_type": doc_type})
            current = _tail(current, overlap)
    if current.strip():
        chunks.append({"text": current.strip(), "source": source, "section": section, "doc_type": doc_type})
    return chunks


def chunk_chat_export(text: str, source: str, chunk_size: int = CHUNK_SIZE, overlap_lines: int = CHAT_OVERLAP_LINES) -> list[dict]:
    """Chunk a DiscordChatExporter-style text export, keeping the channel name as the section."""
    chunks: list[dict] = []
    channel = "unknown"
    current = ""
    for line in text.split("\n"):
        if line.startswith("Channel:"):
            channel = line.split(":", 1)[1].strip()[:100] or channel
        current += line + "\n"
        if len(current) >= chunk_size:
            chunks.append({"text": current.strip(), "source": source, "section": channel, "doc_type": "chat"})
            current = "\n".join(current.split("\n")[-overlap_lines:]) + "\n"
    if current.strip():
        chunks.append({"text": current.strip(), "source": source, "section": channel, "doc_type": "chat"})
    return chunks


# --- Database ---


def connect(path: Path = DB_PATH) -> duckdb.DuckDBPyConnection:
    import duckdb

    conn = duckdb.connect(str(path))
    init_db(conn)
    return conn


def init_db(conn: duckdb.DuckDBPyConnection) -> None:
    import duckdb

    for statement in ("INSTALL vss;", "LOAD vss;"):
        try:
            conn.execute(statement)
        except duckdb.Error:
            pass  # brute-force cosine search still works without the extension
    conn.execute(
        f"""
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
        """
    )
    try:
        conn.execute("CREATE INDEX IF NOT EXISTS idx_chunks_embedding ON chunks USING HNSW(embedding) WITH (metric = 'cosine');")
    except duckdb.Error:
        pass


def file_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()[:16]


def ingest_file(conn: duckdb.DuckDBPyConnection, path: Path, doc_type: str, api_key: str) -> int:
    """Embed and store a file. Unchanged files (same hash) are skipped."""
    fhash = file_hash(path)
    source = path.name
    if conn.execute("SELECT id FROM documents WHERE source = ? AND file_hash = ?", [source, fhash]).fetchone():
        print(f"  {source} unchanged, skipping")
        return 0
    old = conn.execute("SELECT id FROM documents WHERE source = ?", [source]).fetchone()
    if old:
        conn.execute("DELETE FROM chunks WHERE doc_id = ?", [old[0]])
        conn.execute("DELETE FROM documents WHERE id = ?", [old[0]])

    text = path.read_text(encoding="utf-8", errors="replace")
    raw_chunks = chunk_chat_export(text, source) if doc_type == "chat" else chunk_text(text, source, doc_type)
    if not raw_chunks:
        print(f"  {source} produced no chunks")
        return 0

    doc_id = uuid.uuid4().hex[:8]
    conn.execute("INSERT INTO documents (id, source, doc_type, file_hash) VALUES (?, ?, ?, ?)", [doc_id, source, doc_type, fhash])
    print(f"  {source}: {len(raw_chunks)} chunks")
    for start in range(0, len(raw_chunks), BATCH_SIZE):
        batch = raw_chunks[start : start + BATCH_SIZE]
        embeddings = embed_texts([chunk["text"] for chunk in batch], api_key)
        for offset, (chunk, emb) in enumerate(zip(batch, embeddings, strict=True)):
            index = start + offset
            conn.execute(
                "INSERT INTO chunks (id, doc_id, chunk_index, text, section, embedding) VALUES (?, ?, ?, ?, ?, ?)",
                [f"{doc_id}_{index}", doc_id, index, chunk["text"], chunk["section"], emb],
            )
        if start + BATCH_SIZE < len(raw_chunks):
            time.sleep(0.5)  # stay polite to the rate limiter
    conn.commit()
    return len(raw_chunks)


def search(conn: duckdb.DuckDBPyConnection, query: str, api_key: str, limit: int = 5, doc_type: str | None = None) -> list[dict]:
    query_emb = embed_texts([query], api_key)[0]
    type_filter = "AND d.doc_type = ?" if doc_type else ""
    params = [query_emb, doc_type, limit] if doc_type else [query_emb, limit]
    rows = conn.execute(
        f"""
        SELECT c.text, c.section, d.source, d.doc_type,
               array_cosine_similarity(c.embedding, ?::FLOAT[{EMBEDDING_DIMS}]) AS score
        FROM chunks c JOIN documents d ON c.doc_id = d.id
        WHERE c.embedding IS NOT NULL {type_filter}
        ORDER BY score DESC
        LIMIT ?
        """,
        params,
    ).fetchall()
    return [{"text": r[0], "section": r[1], "source": r[2], "doc_type": r[3], "score": round(r[4], 4) if r[4] else 0} for r in rows]


def stats(conn: duckdb.DuckDBPyConnection) -> dict:
    docs = conn.execute("SELECT source, doc_type, file_hash FROM documents ORDER BY source").fetchall()
    total = conn.execute("SELECT COUNT(*) FROM chunks").fetchone()[0]
    return {"documents": [{"source": d[0], "type": d[1], "hash": d[2]} for d in docs], "total_chunks": total}


# --- CLI ---


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="minipupper-docs", description="Semantic search over robot docs and chat exports.")
    parser.add_argument("--db", type=Path, default=DB_PATH, help=f"DuckDB file (default: {DB_PATH}).")
    sub = parser.add_subparsers(dest="command", required=True)

    ingest = sub.add_parser("ingest", help="Embed text files into the database.")
    ingest.add_argument("paths", nargs="+", type=Path)
    ingest.add_argument("--type", choices=DOC_TYPES, default="docs", help="docs (headings) or chat (DiscordChatExporter text).")

    query = sub.add_parser("search", help="Find the chunks closest to a question or error message.")
    query.add_argument("query", nargs="+")
    query.add_argument("--type", choices=DOC_TYPES)
    query.add_argument("--top", type=int, default=5)

    sub.add_parser("stats", help="Show what has been ingested.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    conn = connect(args.db)
    try:
        if args.command == "ingest":
            api_key = get_api_key()
            total = 0
            for path in args.paths:
                if not path.exists():
                    print(f"  {path} not found", file=sys.stderr)
                    continue
                total += ingest_file(conn, path, args.type, api_key)
            s = stats(conn)
            print(f"\n{total} new chunks embedded. Total: {len(s['documents'])} documents, {s['total_chunks']} chunks.")
        elif args.command == "search":
            results = search(conn, " ".join(args.query), get_api_key(), limit=args.top, doc_type=args.type)
            if not results:
                print("No results found.")
            for i, r in enumerate(results, 1):
                print(f"\n{'=' * 60}\nResult {i} | score {r['score']} | {r['source']} ({r['doc_type']})")
                if r["section"]:
                    print(f"Section: {r['section']}")
                print("-" * 60)
                print(r["text"][:500] + (f"\n... ({len(r['text'])} chars total)" if len(r["text"]) > 500 else ""))
        else:
            s = stats(conn)
            print(f"Documents: {len(s['documents'])}")
            for d in s["documents"]:
                print(f"  {d['source']} ({d['type']}) [hash {d['hash']}]")
            print(f"Total chunks: {s['total_chunks']}")
    finally:
        conn.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
