from minipupper_operator import docs_search as ds


def test_chunk_text_tracks_headings_and_overlaps():
    body = "\n".join(f"line {i:03d} " + "x" * 40 for i in range(60))
    text = "# Calibration\n" + body + "\n## SLAM\n" + body
    chunks = ds.chunk_text(text, "docs.txt", chunk_size=800, overlap=200)

    assert len(chunks) > 3
    assert chunks[0]["section"] == "Calibration"
    assert chunks[-1]["section"] == "SLAM"
    assert all(c["source"] == "docs.txt" and c["doc_type"] == "docs" for c in chunks)
    # Consecutive chunks share overlapping lines.
    first_tail = chunks[0]["text"].splitlines()[-1]
    assert first_tail in chunks[1]["text"]


def test_chunk_text_short_document_is_one_chunk():
    chunks = ds.chunk_text("# Title\nhello", "a.txt")
    assert chunks == [{"text": "# Title\nhello", "source": "a.txt", "section": "Title", "doc_type": "docs"}]


def test_chunk_text_empty():
    assert ds.chunk_text("\n\n", "a.txt") == []


def test_chunk_chat_export_uses_channel_name():
    export = "\n".join(
        [
            "==============================================================",
            "Guild: Example Robotics",
            "Channel: help / ros2",
            "==============================================================",
            *[f"[1/1/2024 9:{i:02d} AM] user{i}\nThe lidar says communication is abnormal again." for i in range(40)],
        ]
    )
    chunks = ds.chunk_chat_export(export, "chat.txt", chunk_size=400)
    assert len(chunks) > 2
    assert {c["section"] for c in chunks} == {"help / ros2"}
    assert all(c["doc_type"] == "chat" for c in chunks)


def test_tail_respects_budget():
    assert ds._tail("aaaa\nbb\ncc", 6) == "bb\ncc\n"
    assert ds._tail("a" * 50, 10) == ""


def test_parser_accepts_ingest_and_search():
    args = ds.build_parser().parse_args(["ingest", "--type", "chat", "a.txt", "b.txt"])
    assert args.type == "chat" and [p.name for p in args.paths] == ["a.txt", "b.txt"]
    args = ds.build_parser().parse_args(["search", "servo", "calibration", "--top", "3"])
    assert args.query == ["servo", "calibration"] and args.top == 3
