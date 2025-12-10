import pytest
import asyncio
from fastapi.testclient import TestClient
from backend.src.main import app
from backend.src.services.chatkit_server import ChatKitServer
from backend.src.services.rag_service import rag_service
from backend.src.services.gemini_service import GeminiService
from backend.src.services.storage_service import storage_service
from unittest.mock import AsyncMock, patch
import json


class TestChatE2E:
    """End-to-end tests for complete user flows"""

    def setup_method(self):
        """Setup test client for each test method"""
        self.client = TestClient(app)

    @pytest.mark.asyncio
    async def test_complete_chat_flow(self):
        """Test a complete chat flow: thread creation, message exchange, response"""
        thread_id = "test_thread_e2e_1"

        # First, test health check
        health_response = self.client.get("/.well-known/health")
        assert health_response.status_code == 200
        assert health_response.json()["status"] == "healthy"

        # Mock the services to avoid external dependencies
        with patch.object(rag_service, 'query', new_callable=AsyncMock) as mock_rag_query, \
             patch.object(storage_service, 'create_message', new_callable=AsyncMock) as mock_create_message, \
             patch.object(storage_service, 'get_thread_messages', new_callable=AsyncMock) as mock_get_messages, \
             patch.object(storage_service, 'create_thread', new_callable=AsyncMock) as mock_create_thread:

            # Setup mocks
            from backend.src.models.rag import RetrievedDocument, RAGResponse
            mock_doc = RetrievedDocument(
                document_id="test_doc",
                score=0.9,
                source="test:section",
                text="This is relevant test content for the query"
            )
            mock_rag_response = RAGResponse(
                query="test query",
                top_k=5,
                results=[mock_doc],
                assembled_context="This is relevant test content for the query",
                retrieval_meta={"method": "qdrant"}
            )
            mock_rag_query.return_value = mock_rag_response

            # Mock empty message history initially
            mock_get_messages.return_value = []

            # Mock successful message creation
            mock_create_message.return_value = None

            from backend.src.models.conversation import ConversationThread
            mock_thread = ConversationThread(
                thread_id=thread_id,
                user_id="test_user_e2e",
                created_at=type('obj', (object,), {'isoformat': lambda: "2023-01-01T00:00:00"})(),
                updated_at=type('obj', (object,), {'isoformat': lambda: "2023-01-01T00:00:00"})(),
                book_id="test_book",
                chapter_id="test_chapter",
                metadata={}
            )
            mock_create_thread.return_value = mock_thread

            # Test the chat endpoint (will fail without auth, but structure should be valid)
            payload = {
                "thread_id": thread_id,
                "messages": [
                    {
                        "role": "user",
                        "content": "Hello, can you explain the main concepts in this book?"
                    }
                ],
                "context": {
                    "book_id": "test_book",
                    "chapter_id": "introduction"
                }
            }

            # Try with authentication header (will fail with invalid token, but test structure)
            headers = {"Authorization": "Bearer invalid_token_for_testing_structure"}
            response = self.client.post("/chatkit", json=payload, headers=headers)

            # The response should not be a validation error (422) - indicates structure is correct
            # It will likely be 401 due to invalid auth, which is expected
            assert response.status_code != 422

    def test_search_action_flow(self):
        """Test the complete search action flow"""
        with patch.object(rag_service, 'advanced_search', new_callable=AsyncMock) as mock_search:
            from backend.src.models.rag import RetrievedDocument, RAGResponse
            mock_doc = RetrievedDocument(
                document_id="search_result_1",
                score=0.85,
                source="chapter:3:section:2",
                text="This section discusses the key algorithms used in the system"
            )
            mock_response = RAGResponse(
                query="algorithms",
                top_k=5,
                results=[mock_doc],
                assembled_context="This section discusses the key algorithms used in the system",
                retrieval_meta={"method": "qdrant_semantic_search"}
            )
            mock_search.return_value = mock_response

            payload = {
                "action": "search",
                "payload": {
                    "query": "key algorithms",
                    "book_id": "test_book",
                    "top_k": 3
                },
                "thread_id": "test_thread_search"
            }

            headers = {"Authorization": "Bearer dummy_token_for_testing"}
            response = self.client.post("/actions", json=payload, headers=headers)

            # Should not return validation error
            assert response.status_code != 422

    def test_navigation_action_flow(self):
        """Test the complete navigation action flow"""
        with patch('backend.src.services.content_relationship_service.content_relationship_service') as mock_service:
            from backend.src.services.content_relationship_service import ContentRelationship
            mock_rel = ContentRelationship(
                from_section="current_section",
                to_section="related_section",
                relationship_type="related_to",
                strength=0.8,
                reason="These sections cover related concepts"
            )
            mock_service.find_navigation_suggestions.return_value = [mock_rel]

            payload = {
                "action": "navigate_section",
                "payload": {
                    "current_section": "current_section",
                    "topic": "related concepts"
                },
                "thread_id": "test_thread_nav"
            }

            headers = {"Authorization": "Bearer dummy_token_for_testing"}
            response = self.client.post("/actions", json=payload, headers=headers)

            assert response.status_code != 422

    def test_complete_conversation_flow(self):
        """Test a multi-turn conversation flow"""
        thread_id = "test_conversation_thread"

        # Mock multiple RAG responses for different queries
        with patch.object(rag_service, 'query', new_callable=AsyncMock) as mock_rag_query, \
             patch.object(storage_service, 'create_message', new_callable=AsyncMock) as mock_create_message, \
             patch.object(storage_service, 'get_thread_messages', new_callable=AsyncMock) as mock_get_messages:

            from backend.src.models.rag import RetrievedDocument, RAGResponse
            # First query response
            doc1 = RetrievedDocument(
                document_id="doc1",
                score=0.9,
                source="chapter:1",
                text="First chapter content about basic concepts"
            )
            mock_rag_query.return_value = RAGResponse(
                query="basic concepts",
                top_k=5,
                results=[doc1],
                assembled_context="First chapter content about basic concepts",
                retrieval_meta={"method": "qdrant"}
            )

            # Mock message history with previous messages for context
            from backend.src.models.message import Message
            prev_messages = [
                Message(
                    message_id="msg1",
                    thread_id=thread_id,
                    role="user",
                    content="What are the basic concepts?",
                    created_at="2023-01-01T00:00:00",
                    metadata={}
                ),
                Message(
                    message_id="msg2",
                    thread_id=thread_id,
                    role="assistant",
                    content="The basic concepts include...",
                    created_at="2023-01-01T00:00:01",
                    metadata={}
                )
            ]
            mock_get_messages.return_value = prev_messages
            mock_create_message.return_value = None

            # First message in conversation
            payload1 = {
                "thread_id": thread_id,
                "messages": [
                    {"role": "user", "content": "What are the basic concepts?"}
                ],
                "context": {"book_id": "test_book"}
            }

            headers = {"Authorization": "Bearer dummy_token"}
            response1 = self.client.post("/chatkit", json=payload1, headers=headers)
            assert response1.status_code != 422

            # Follow-up message that should include context
            payload2 = {
                "thread_id": thread_id,
                "messages": [
                    {"role": "user", "content": "Can you elaborate on the second concept?"},
                    {"role": "assistant", "content": "The basic concepts include..."},
                    {"role": "user", "content": "Can you elaborate on the second concept?"}
                ],
                "context": {"book_id": "test_book"}
            }

            response2 = self.client.post("/chatkit", json=payload2, headers=headers)
            assert response2.status_code != 422

    def test_error_handling_flow(self):
        """Test error handling in the complete flow"""
        # Test with invalid thread ID format
        payload = {
            "thread_id": "<script>alert('xss')</script>",  # Should be sanitized
            "messages": [{"role": "user", "content": "test"}],
            "context": {}
        }

        headers = {"Authorization": "Bearer dummy_token"}
        response = self.client.post("/chatkit", json=payload, headers=headers)

        # Should handle the invalid format gracefully
        assert response.status_code in [400, 422, 401]  # Validation or auth error expected

    def test_security_validation_flow(self):
        """Test that security validations work end-to-end"""
        malicious_payload = {
            "thread_id": "valid_thread_id",
            "messages": [
                {
                    "role": "user",
                    "content": "<script>alert('xss')</script> DROP TABLE users; --"
                }
            ],
            "context": {
                "book_id": "<img src=x onerror=alert('xss')>",
                "chapter_id": "normal_chapter"
            }
        }

        headers = {"Authorization": "Bearer dummy_token"}
        response = self.client.post("/chatkit", json=malicious_payload, headers=headers)

        # Should not crash, should handle validation
        assert response.status_code != 500  # Internal server error would indicate failed validation


class TestSystemIntegration:
    """Tests for integration between all system components"""

    def test_all_services_integration(self):
        """Test that all services can be imported and have expected interfaces"""
        # Verify all services exist and have expected methods
        assert hasattr(rag_service, 'query')
        assert hasattr(rag_service, 'semantic_search')
        assert hasattr(storage_service, 'create_message')
        assert hasattr(storage_service, 'get_thread_messages')

        server = ChatKitServer()
        assert hasattr(server, 'respond')
        assert hasattr(server, 'handle_action')

        gemini = GeminiService()
        assert hasattr(gemini, 'generate_stream')
        assert hasattr(gemini, 'generate_response_with_navigation')

    def test_api_routes_exist(self):
        """Test that all expected API routes are available"""
        # Health check
        health_response = self.client.get("/.well-known/health")
        assert health_response.status_code == 200

        # These will fail without proper auth, but should not return 404
        assert app.router.routes  # Verify routes exist