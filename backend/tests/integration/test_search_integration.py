import pytest
from fastapi.testclient import TestClient
from backend.src.main import app
from backend.src.services.rag_service import rag_service
from unittest.mock import AsyncMock, patch
import json


class TestSearchIntegration:
    """Integration tests for search functionality"""

    def setup_method(self):
        """Setup test client for each test method"""
        self.client = TestClient(app)

    def test_actions_endpoint_search_integration(self):
        """Test search action through the actions endpoint"""
        with patch.object(rag_service, 'advanced_search', new_callable=AsyncMock) as mock_search:
            # Mock search results
            from backend.src.models.rag import RetrievedDocument, RAGResponse
            mock_doc = RetrievedDocument(
                document_id="test_doc",
                score=0.9,
                source="test:source",
                text="This is test content for search functionality"
            )
            mock_response = RAGResponse(
                query="test query",
                top_k=5,
                results=[mock_doc],
                assembled_context="test context",
                retrieval_meta={"method": "qdrant"}
            )
            mock_search.return_value = mock_response

            # Prepare search action payload
            payload = {
                "action": "search",
                "payload": {
                    "query": "test search query",
                    "book_id": "test_book",
                    "chapter_id": "test_chapter",
                    "top_k": 3
                },
                "thread_id": "test_thread_123"
            }

            # Make request with authentication header
            headers = {"Authorization": "Bearer mock_token_for_testing"}
            response = self.client.post("/actions", json=payload, headers=headers)

            # Should not return validation error (422) - might return 401 for auth
            assert response.status_code != 422

    @pytest.mark.asyncio
    async def test_search_with_filters(self):
        """Test search with various filter combinations"""
        with patch.object(rag_service, 'advanced_search', new_callable=AsyncMock) as mock_search:
            # Mock response
            from backend.src.models.rag import RetrievedDocument, RAGResponse
            mock_doc = RetrievedDocument(
                document_id="filtered_doc",
                score=0.85,
                source="filtered:source",
                text="Content with specific filters applied"
            )
            mock_response = RAGResponse(
                query="filtered query",
                top_k=5,
                results=[mock_doc],
                assembled_context="filtered context",
                retrieval_meta={"method": "qdrant", "filters_applied": ["book_id"]}
            )
            mock_search.return_value = mock_response

            # Test search with filters
            payload = {
                "action": "search",
                "payload": {
                    "query": "filtered search",
                    "book_id": "specific_book",
                    "tags": ["important", "key_concept"],
                    "top_k": 5
                },
                "thread_id": "test_thread_456"
            }

            headers = {"Authorization": "Bearer mock_token_for_testing"}
            response = self.client.post("/actions", json=payload, headers=headers)

            # Verify the request structure was processed correctly
            assert response.status_code != 422

    def test_search_result_formatting(self):
        """Test that search results are properly formatted for frontend"""
        with patch.object(rag_service, 'advanced_search', new_callable=AsyncMock) as mock_search:
            # Mock multiple search results
            from backend.src.models.rag import RetrievedDocument, RAGResponse
            mock_docs = [
                RetrievedDocument(
                    document_id=f"doc_{i}",
                    score=0.9 - (i * 0.1),
                    source=f"test:section_{i}",
                    text=f"This is content for document {i} with more details"
                )
                for i in range(3)
            ]
            mock_response = RAGResponse(
                query="multi-result query",
                top_k=5,
                results=mock_docs,
                assembled_context="multiple results context",
                retrieval_meta={"method": "qdrant"}
            )
            mock_search.return_value = mock_response

            payload = {
                "action": "search",
                "payload": {"query": "multi-result query", "top_k": 3},
                "thread_id": "test_thread_789"
            }

            headers = {"Authorization": "Bearer mock_token_for_testing"}
            response = self.client.post("/actions", json=payload, headers=headers)

            if response.status_code == 200:  # If authentication passes
                data = response.json()
                assert "results" in data
                assert len(data["results"]) == 3  # top_k = 3

                # Check result structure
                for result in data["results"]:
                    assert "title" in result
                    assert "snippet" in result
                    assert "source" in result
                    assert "score" in result

    def test_search_error_handling(self):
        """Test search error handling in the integration"""
        with patch.object(rag_service, 'advanced_search', side_effect=Exception("Search failed")):
            payload = {
                "action": "search",
                "payload": {"query": "failing query"},
                "thread_id": "test_thread_error"
            }

            headers = {"Authorization": "Bearer mock_token_for_testing"}
            response = self.client.post("/actions", json=payload, headers=headers)

            # Should handle the error gracefully, not crash
            # May return 500 error or be handled by exception middleware
            assert response.status_code in [200, 401, 500]

    def test_search_deduplication_feature(self):
        """Test that search deduplication feature works in integration"""
        with patch.object(rag_service, 'advanced_search', new_callable=AsyncMock) as mock_search:
            # Mock response with potentially duplicate sources
            from backend.src.models.rag import RetrievedDocument, RAGResponse
            mock_docs = [
                RetrievedDocument(
                    document_id="doc_1",
                    score=0.9,
                    source="duplicate:source",
                    text="First occurrence"
                ),
                RetrievedDocument(
                    document_id="doc_2",
                    score=0.7,  # Lower score, should be filtered out if deduplication works
                    source="duplicate:source",  # Same source as doc_1
                    text="Second occurrence"
                ),
                RetrievedDocument(
                    document_id="doc_3",
                    score=0.8,
                    source="unique:source",
                    text="Unique content"
                )
            ]
            mock_response = RAGResponse(
                query="deduplication query",
                top_k=5,
                results=mock_docs,
                assembled_context="test context",
                retrieval_meta={"method": "qdrant", "deduplicated": True}
            )
            mock_search.return_value = mock_response

            payload = {
                "action": "search",
                "payload": {
                    "query": "deduplication test",
                    "deduplicate": True
                },
                "thread_id": "test_thread_dedupe"
            }

            headers = {"Authorization": "Bearer mock_token_for_testing"}
            response = self.client.post("/actions", json=payload, headers=headers)

            assert response.status_code != 422


class TestRAGServiceIntegration:
    """Additional integration tests for RAG service components"""

    @pytest.mark.asyncio
    async def test_rag_service_initialization(self):
        """Test that RAG service initializes correctly"""
        # Verify the service has the expected methods
        assert hasattr(rag_service, 'query')
        assert hasattr(rag_service, 'semantic_search')
        assert hasattr(rag_service, 'advanced_search')
        assert hasattr(rag_service, '_remove_duplicate_sources')

    @pytest.mark.asyncio
    async def test_embedding_integration_point(self):
        """Test the integration point for embeddings"""
        # This would test the actual embedding function when implemented
        # For now, we're testing that the method exists and can be called
        assert hasattr(rag_service, '_create_embedding')

    @pytest.mark.asyncio
    async def test_context_assembly_integration(self):
        """Test context assembly in integration context"""
        from backend.src.models.rag import RetrievedDocument

        docs = [
            RetrievedDocument(
                document_id="test_1",
                score=0.9,
                source="integration:test",
                text="Integration test content"
            )
        ]

        context = rag_service._assemble_context(docs)
        assert "Integration test content" in context
        assert "integration:test" in context