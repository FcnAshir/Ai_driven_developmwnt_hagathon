import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from backend.src.services.rag_service import RAGService, rag_service
from backend.src.models.rag import RetrievedDocument, RAGResponse


class TestSearchFunctionality:
    """Unit tests for search functionality"""

    @pytest.mark.asyncio
    async def test_semantic_search_basic(self):
        """Test basic semantic search functionality"""
        # Mock the embedding function
        with patch.object(rag_service, '_create_embedding', new_callable=AsyncMock) as mock_embedding:
            mock_embedding.return_value = [0.1] * 1536  # Mock embedding vector

            # Mock the Qdrant client search method
            with patch.object(rag_service.client, 'search') as mock_search:
                # Mock search result
                mock_hit = MagicMock()
                mock_hit.id = "doc1"
                mock_hit.score = 0.9
                mock_hit.payload = {
                    "text": "This is a test document",
                    "source": "test:section1",
                    "cursor": 100
                }
                mock_search.return_value = [mock_hit]

                # Perform semantic search
                result = await rag_service.semantic_search("test query")

                # Assertions
                assert isinstance(result, RAGResponse)
                assert result.query == "test query"
                assert len(result.results) == 1
                assert result.results[0].document_id == "doc1"
                assert result.results[0].score == 0.9
                assert result.results[0].text == "This is a test document"

    @pytest.mark.asyncio
    async def test_semantic_search_with_filters(self):
        """Test semantic search with filters"""
        with patch.object(rag_service, '_create_embedding', new_callable=AsyncMock) as mock_embedding:
            mock_embedding.return_value = [0.1] * 1536

            with patch.object(rag_service.client, 'search') as mock_search:
                # Mock search result
                mock_hit = MagicMock()
                mock_hit.id = "doc2"
                mock_hit.score = 0.85
                mock_hit.payload = {
                    "text": "Filtered document content",
                    "source": "test:section2",
                    "book_id": "test_book",
                    "chapter_id": "test_chapter"
                }
                mock_search.return_value = [mock_hit]

                # Perform search with filters
                filters = {
                    "book_id": "test_book",
                    "chapter_id": "test_chapter"
                }
                result = await rag_service.semantic_search("test query", filters=filters)

                # Assertions
                assert result.query == "test query"
                assert len(result.results) == 1
                assert result.results[0].text == "Filtered document content"
                assert "qdrant_semantic_search" in result.retrieval_meta["method"]

    @pytest.mark.asyncio
    async def test_advanced_search(self):
        """Test advanced search functionality"""
        with patch.object(rag_service, '_create_embedding', new_callable=AsyncMock) as mock_embedding:
            mock_embedding.return_value = [0.1] * 1536

            with patch.object(rag_service.client, 'search') as mock_search:
                # Mock search result
                mock_hit = MagicMock()
                mock_hit.id = "doc3"
                mock_hit.score = 0.95
                mock_hit.payload = {
                    "text": "Advanced search result",
                    "source": "test:section3"
                }
                mock_search.return_value = [mock_hit]

                result = await rag_service.advanced_search("advanced query")

                assert isinstance(result, RAGResponse)
                assert result.query == "advanced query"
                assert len(result.results) == 1
                assert result.results[0].text == "Advanced search result"

    def test_remove_duplicate_sources(self):
        """Test duplicate source removal functionality"""
        docs = [
            RetrievedDocument(
                document_id="doc1",
                score=0.9,
                source="source_a",
                text="content 1"
            ),
            RetrievedDocument(
                document_id="doc2",
                score=0.8,
                source="source_a",  # Same source as doc1
                text="content 2"
            ),
            RetrievedDocument(
                document_id="doc3",
                score=0.95,
                source="source_b",
                text="content 3"
            )
        ]

        # Should keep the one with higher score for each source
        result = rag_service._remove_duplicate_sources(docs)

        assert len(result) == 2  # Two unique sources
        # Should keep the one with highest score for source_a (doc1 with score 0.9)
        source_a_docs = [doc for doc in result if doc.source == "source_a"]
        assert len(source_a_docs) == 1
        assert source_a_docs[0].document_id == "doc1"  # Higher score
        assert source_a_docs[0].score == 0.9

    def test_assemble_context(self):
        """Test context assembly functionality"""
        docs = [
            RetrievedDocument(
                document_id="doc1",
                score=0.9,
                source="source_a",
                text="First document content"
            ),
            RetrievedDocument(
                document_id="doc2",
                score=0.8,
                source="source_b",
                text="Second document content"
            )
        ]

        context = rag_service._assemble_context(docs)

        # Check that both sources and contents are included
        assert "Source: source_a" in context
        assert "First document content" in context
        assert "Source: source_b" in context
        assert "Second document content" in context
        assert "---" in context  # Separator

    @pytest.mark.asyncio
    async def test_semantic_search_error_handling(self):
        """Test error handling in semantic search"""
        with patch.object(rag_service, '_create_embedding', side_effect=Exception("Embedding error")):
            result = await rag_service.semantic_search("test query")

            # Should return empty results with error in metadata
            assert len(result.results) == 0
            assert result.assembled_context == ""
            assert "error" in result.retrieval_meta

    @pytest.mark.asyncio
    async def test_advanced_search_error_handling(self):
        """Test error handling in advanced search"""
        with patch.object(rag_service, 'semantic_search', side_effect=Exception("Search error")):
            result = await rag_service.advanced_search("test query")

            # Should return empty results with error in metadata
            assert len(result.results) == 0
            assert result.assembled_context == ""
            assert "error" in result.retrieval_meta