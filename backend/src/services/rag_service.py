from typing import List, Optional, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from ..models.rag import RetrievedDocument, RAGRequest, RAGResponse
from ..config import settings
import logging

logger = logging.getLogger(__name__)

class RAGService:
    """
    Service for interacting with the RAG (Retrieval-Augmented Generation) system
    using Qdrant vector database
    """

    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False  # Set to True in production for better performance
        )
        self.collection_name = "book_content"
        self.top_k = settings.top_k
        self.max_context_tokens = settings.max_context_tokens

    async def initialize_collection(self):
        """
        Initialize the Qdrant collection if it doesn't exist
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                # Create collection with vector size appropriate for embeddings
                # Using 1536 dimensions which is common for text-embedding-ada-002
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")
        except Exception as e:
            logger.error(f"Error initializing Qdrant collection: {e}")
            raise

    async def query(self, query_text: str, context_filters: Optional[Dict[str, Any]] = None) -> RAGResponse:
        """
        Query the RAG system to retrieve relevant documents based on the input query
        """
        logger.info(f"RAG query started: '{query_text[:50]}...' with filters: {bool(context_filters)}")

        try:
            # Create embedding for the query (in a real implementation, you would use an embedding model)
            # For now, we'll simulate this with a placeholder
            query_embedding = await self._create_embedding(query_text)

            # Prepare filters based on context
            filters = []
            if context_filters:
                logger.debug(f"Applying context filters: {context_filters}")
                if context_filters.get("book_id"):
                    filters.append(models.FieldCondition(
                        key="book_id",
                        match=models.MatchValue(value=context_filters["book_id"])
                    ))
                if context_filters.get("chapter_id"):
                    filters.append(models.FieldCondition(
                        key="chapter_id",
                        match=models.MatchValue(value=context_filters["chapter_id"])
                    ))

            # Perform search in Qdrant
            logger.debug(f"Performing Qdrant search with {len(filters)} filters")
            search_result = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=self.top_k,
                query_filter=models.Filter(must=filters) if filters else None,
                with_payload=True
            )

            # Convert search results to RetrievedDocument objects
            results = []
            for hit in search_result:
                payload = hit.payload
                results.append(RetrievedDocument(
                    document_id=hit.id,
                    score=hit.score,
                    source=payload.get("source", ""),
                    text=payload.get("text", ""),
                    cursor=payload.get("cursor", None)
                ))

            logger.info(f"RAG query returned {len(results)} results")

            # Assemble context from retrieved documents
            assembled_context = self._assemble_context(results)

            response = RAGResponse(
                query=query_text,
                top_k=self.top_k,
                results=results,
                assembled_context=assembled_context,
                retrieval_meta={
                    "method": "qdrant",
                    "params": {
                        "top_k": self.top_k,
                        "filters": [f.key for f in filters] if filters else [],
                        "query_length": len(query_text)
                    }
                }
            )

            logger.debug(f"RAG query completed successfully, context length: {len(assembled_context)} chars")
            return response

        except Exception as e:
            logger.error(f"Error querying RAG system: {e}", exc_info=True)
            # Return empty results in case of error
            return RAGResponse(
                query=query_text,
                top_k=self.top_k,
                results=[],
                assembled_context="",
                retrieval_meta={"error": str(e)}
            )

    async def _create_embedding(self, text: str) -> List[float]:
        """
        Create embedding for the given text
        In a real implementation, this would call an embedding model API
        For now, returning a placeholder embedding
        """
        # Placeholder implementation - in real system, use an actual embedding model
        # For example, using OpenAI embeddings or a local model
        logger.warning("Using placeholder embedding function - replace with actual embedding model")
        # Return a vector of 1536 zeros as a placeholder
        return [0.0] * 1536

    def _assemble_context(self, retrieved_docs: List[RetrievedDocument]) -> str:
        """
        Assemble the context string from retrieved documents
        """
        context_parts = []
        for doc in retrieved_docs:
            context_parts.append(f"Source: {doc.source}\nContent: {doc.text}\n---")

        full_context = "\n".join(context_parts)

        # Truncate if too long
        if len(full_context) > self.max_context_tokens * 4:  # Rough estimate: 4 chars per token
            full_context = full_context[:self.max_context_tokens * 4]

        return full_context

    async def semantic_search(self, query: str, filters: Optional[Dict[str, Any]] = None, top_k: Optional[int] = None) -> RAGResponse:
        """
        Perform semantic search with enhanced capabilities
        """
        try:
            # Create embedding for the query
            query_embedding = await self._create_embedding(query)

            # Prepare filters
            qdrant_filters = []
            if filters:
                if filters.get("book_id"):
                    qdrant_filters.append(models.FieldCondition(
                        key="book_id",
                        match=models.MatchValue(value=filters["book_id"])
                    ))
                if filters.get("chapter_id"):
                    qdrant_filters.append(models.FieldCondition(
                        key="chapter_id",
                        match=models.MatchValue(value=filters["chapter_id"])
                    ))
                if filters.get("tags"):
                    # Assuming tags are stored as a separate field
                    for tag in filters["tags"]:
                        qdrant_filters.append(models.FieldCondition(
                            key="tags",
                            match=models.MatchAny(any=[tag])
                        ))

            # Perform search in Qdrant
            search_result = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k or self.top_k,
                query_filter=models.Filter(must=qdrant_filters) if qdrant_filters else None,
                with_payload=True,
                # Use score_threshold if provided in filters
                score_threshold=filters.get("min_score", 0.0) if filters else 0.0
            )

            # Convert search results to RetrievedDocument objects
            results = []
            for hit in search_result:
                payload = hit.payload
                results.append(RetrievedDocument(
                    document_id=hit.id,
                    score=hit.score,
                    source=payload.get("source", ""),
                    text=payload.get("text", ""),
                    cursor=payload.get("cursor", None)
                ))

            # Sort results by score if needed
            results = sorted(results, key=lambda x: x.score, reverse=True)

            # Apply additional post-processing if needed
            if filters and filters.get("deduplicate"):
                results = self._remove_duplicate_sources(results)

            # Assemble context from retrieved documents
            assembled_context = self._assemble_context(results)

            return RAGResponse(
                query=query,
                top_k=top_k or self.top_k,
                results=results,
                assembled_context=assembled_context,
                retrieval_meta={
                    "method": "qdrant_semantic_search",
                    "params": {
                        "top_k": top_k or self.top_k,
                        "filters_applied": [f.key for f in qdrant_filters] if qdrant_filters else [],
                        "query_embedding_length": len(query_embedding) if query_embedding else 0
                    }
                }
            )
        except Exception as e:
            logger.error(f"Error in semantic search: {e}")
            return RAGResponse(
                query=query,
                top_k=top_k or self.top_k,
                results=[],
                assembled_context="",
                retrieval_meta={"error": str(e)}
            )

    def _remove_duplicate_sources(self, docs: List[RetrievedDocument]) -> List[RetrievedDocument]:
        """
        Remove documents with duplicate sources, keeping the one with highest score
        """
        unique_docs = {}
        for doc in docs:
            source = doc.source
            if source not in unique_docs or doc.score > unique_docs[source].score:
                unique_docs[source] = doc

        return list(unique_docs.values())

    async def advanced_search(self, query: str, filters: Optional[Dict[str, Any]] = None) -> RAGResponse:
        """
        Advanced search with multiple search strategies combined
        """
        try:
            # For now, we'll use the semantic search as the primary method
            # In a more advanced implementation, we could combine multiple strategies
            return await self.semantic_search(query, filters)
        except Exception as e:
            logger.error(f"Error in advanced search: {e}")
            return RAGResponse(
                query=query,
                top_k=self.top_k,
                results=[],
                assembled_context="",
                retrieval_meta={"error": str(e)}
            )

    async def add_document(self, doc_id: str, text: str, book_id: str, chapter_id: str, embedding: List[float] = None):
        """
        Add a document to the Qdrant collection
        """
        if embedding is None:
            embedding = await self._create_embedding(text)

        point = models.PointStruct(
            id=doc_id,
            vector=embedding,
            payload={
                "text": text,
                "book_id": book_id,
                "chapter_id": chapter_id,
                "source": f"{book_id}:{chapter_id}"
            }
        )

        self.client.upsert(
            collection_name=self.collection_name,
            points=[point]
        )

# Global instance
rag_service = RAGService()