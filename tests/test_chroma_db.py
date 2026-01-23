from bandu.rag.rag_client import RagChroma


def test_chroma_db():
    rag_chroma = RagChroma("test_chroma")
    # ensure a clean collection for the test
    rag_chroma.reset_collection()

    # add documents (use add_texts because add_documents expects Document objects)
    rag_chroma.add_texts(
        [
            "The location of Kitchen is x: 5.0, y: 3.0",
            "The location of Bedroom is x: 2.0, y: 2.0",
            "The location of Living room is x: 10.0, y: 7.5",
        ]
    )

    # query the collection for the Kitchen location
    results = rag_chroma.similarity_search("What is the location of Kitchen?", k=1)

    assert results, "Expected at least one result from similarity_search"
    top = results[0].page_content

    # verify the top result mentions Kitchen and the expected coordinates
    assert "Kitchen" in top
    assert "x: 5.0" in top
    assert "y: 3.0" in top

    # cleanup
    rag_chroma.reset_collection()
