import pytest
import asyncio
from fastapi.testclient import TestClient
from backend.src.main import app
from backend.src.services.chatkit_server import ChatKitServer
from backend.src.services.gemini_service import GeminiService
from backend.src.services.rag_service import rag_service
from backend.src.services.storage_service import storage_service
from unittest.mock import AsyncMock, patch
import json


class TestChatIntegration:
    """Integration tests for chat functionality"""

    def setup_method(self):
        """Setup test client for each test method"""
        self.client = TestClient(app)

    @pytest.mark.asyncio
    async def test_health_check_endpoint(self):
        """Test that health check endpoint returns healthy status"""
        response = self.client.get("/.well-known/health")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "healthy"
        assert "timestamp" in data
        assert data["service"] == "chatkit-gemini-rag-integration"

    @pytest.mark.asyncio
    async def test_chatkit_endpoint_basic_functionality(self):
        """Test basic chat functionality through the chatkit endpoint"""
        # Mock the services to avoid external dependencies during testing
        with patch.object(rag_service, 'query', new_callable=AsyncMock) as mock_rag_query, \
             patch.object(storage_service, 'create_message', new_callable=AsyncMock) as mock_create_message, \
             patch.object(storage_service, 'get_thread_messages', new_callable=AsyncMock) as mock_get_messages:

            # Mock RAG response
            mock_rag_query.return_value.assembled_context = "Test context from RAG"

            # Mock empty message history
            mock_get_messages.return_value = []

            # Mock storage service
            mock_create_message.return_value = None

            # Prepare test data
            payload = {
                "thread_id": "test_thread_123",
                "messages": [
                    {
                        "role": "user",
                        "content": "Hello, can you help me with the book content?"
                    }
                ],
                "context": {
                    "book_id": "test_book",
                    "chapter_id": "test_chapter"
                }
            }

            # Create a mock JWT token for authentication
            # In a real test, you would create a proper JWT token
            headers = {
                "Authorization": "Bearer mock_token_for_testing"
            }

            # This test would need to handle the streaming response
            # For now, we'll test the endpoint structure
            response = self.client.post("/chatkit", json=payload, headers=headers)

            # Note: Since this endpoint streams responses, we may get different status codes
            # depending on how the authentication and initial processing works
            # The actual response content would be tested differently for streaming endpoints

    @pytest.mark.asyncio
    async def test_chatkit_endpoint_missing_messages(self):
        """Test chatkit endpoint with missing messages"""
        payload = {
            "thread_id": "test_thread_123",
            "messages": [],
            "context": {}
        }

        headers = {
            "Authorization": "Bearer mock_token_for_testing"
        }

        response = self.client.post("/chatkit", json=payload, headers=headers)
        assert response.status_code == 400
        assert "No messages provided" in response.text

    @pytest.mark.asyncio
    async def test_chatkit_endpoint_invalid_last_message(self):
        """Test chatkit endpoint with invalid last message (not from user)"""
        payload = {
            "thread_id": "test_thread_123",
            "messages": [
                {
                    "role": "assistant",
                    "content": "This should not be the last message"
                }
            ],
            "context": {}
        }

        headers = {
            "Authorization": "Bearer mock_token_for_testing"
        }

        response = self.client.post("/chatkit", json=payload, headers=headers)
        assert response.status_code == 400
        assert "Last message must be from user" in response.text

    @pytest.mark.asyncio
    async def test_actions_endpoint_search(self):
        """Test actions endpoint with search action"""
        with patch.object(rag_service, 'query', new_callable=AsyncMock) as mock_rag_query:
            # Mock RAG response
            mock_rag_query.return_value.results = []

            payload = {
                "action": "search",
                "payload": {
                    "query": "test search query",
                    "book_id": "test_book"
                },
                "thread_id": "test_thread_123"
            }

            headers = {
                "Authorization": "Bearer mock_token_for_testing"
            }

            response = self.client.post("/actions", json=payload, headers=headers)
            # This would fail without proper JWT validation in a real scenario
            # The test is structured to show how it would work

    @pytest.mark.asyncio
    async def test_storage_service_integration(self):
        """Test storage service integration"""
        # Test creating a thread
        from backend.src.models.conversation import ConversationThreadCreate

        thread_data = ConversationThreadCreate(
            user_id="test_user",
            book_id="test_book",
            chapter_id="test_chapter",
            metadata={"test": True}
        )

        # This would require a real database connection
        # For testing purposes, we're verifying the structure
        assert thread_data.user_id == "test_user"
        assert thread_data.book_id == "test_book"


# Additional integration tests would go here
# These would include:
# - Full end-to-end chat flows
# - Database persistence verification
# - RAG system integration
# - LLM service integration
# - Authentication flow validation
# - Rate limiting validation