"""
Rag Client
"""

from langchain_chroma import Chroma
from rai import get_embeddings_model


class RagChroma(Chroma):
    def __init__(self, collection_name: str):
        embedded_model = get_embeddings_model()
        super().__init__(
            collection_name=collection_name,
            embedding_function=embedded_model,
            persist_directory="persistent_db",
        )

    def run_query(self, query: str) -> str:
        """
        Run query
        """
        result = self.similarity_search(query, 1)

        return result[0].page_content
