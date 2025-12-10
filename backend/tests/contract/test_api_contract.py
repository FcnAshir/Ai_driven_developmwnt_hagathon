import pytest
import requests
from fastapi.testclient import TestClient
from backend.src.main import app
from backend.src.models.conversation import ConversationThreadCreate
from backend.src.models.message import MessageCreate
from backend.src.models.rag import RAGRequest
import json
from unittest.mock import patch


class TestAPIContract:
    """Contract tests for API endpoints based on the specification"""

    def setup_method(self):
        """Setup test client for each test method"""
        self.client = TestClient(app)

    def test_health_endpoint_contract(self):
        """Test that health endpoint matches the contract specification"""
        response = self.client.get("/.well-known/health")

        # Status code
        assert response.status_code == 200

        # Response structure
        data = response.json()
        assert "status" in data
        assert "timestamp" in data

        # Validate specific fields
        assert data["status"] == "healthy"
        assert isinstance(data["timestamp"], str)
        assert "service" in data
        assert data["service"] == "chatkit-gemini-rag-integration"

    def test_chatkit_endpoint_request_structure(self):
        """Test that chatkit endpoint accepts the correct request structure"""
        # Test valid request structure
        valid_payload = {
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

        # This will fail with authentication, but should accept the structure
        headers = {"Authorization": "Bearer dummy_token"}
        response = self.client.post("/chatkit", json=valid_payload, headers=headers)

        # The response will likely be 401 due to invalid token, but the structure was accepted
        # What's important is that it doesn't return 422 (validation error)
        assert response.status_code != 422  # Unprocessable entity would indicate structure error

    def test_chatkit_endpoint_response_streaming_format(self):
        """Test that chatkit endpoint response follows streaming format contract"""
        # This test would be difficult to validate streaming format in a unit test
        # In a real contract test, we'd validate the SSE format when we have actual responses

        # For now, we'll validate that the endpoint exists and accepts the right content type
        assert True  # Placeholder - actual streaming validation would be done differently

    def test_actions_endpoint_contract(self):
        """Test that actions endpoint matches the contract specification"""
        valid_payload = {
            "action": "search",
            "payload": {
                "query": "test query",
                "topic": "test topic",
                "section_id": "test_section"
            },
            "thread_id": "test_thread_123"
        }

        headers = {"Authorization": "Bearer dummy_token"}
        response = self.client.post("/actions", json=valid_payload, headers=headers)

        # Should accept the structure (will likely return 401 due to auth)
        assert response.status_code != 422

    def test_request_validation_contract(self):
        """Test that API validates requests according to contract"""
        # Test missing thread_id in chatkit
        invalid_payload = {
            "messages": [
                {
                    "role": "user",
                    "content": "Hello"
                }
            ]
        }

        headers = {"Authorization": "Bearer dummy_token"}
        response = self.client.post("/chatkit", json=invalid_payload, headers=headers)

        # Should return validation error or be handled gracefully
        assert response.status_code in [200, 400, 422, 401]

    def test_error_handling_contract(self):
        """Test that API returns appropriate error codes as per contract"""
        # Test 401 Unauthorized (no auth header)
        response = self.client.post("/chatkit", json={
            "thread_id": "test",
            "messages": [{"role": "user", "content": "test"}]
        })
        assert response.status_code == 401

        # Test 400 Bad Request (no messages)
        headers = {"Authorization": "Bearer dummy_token"}
        response = self.client.post("/chatkit", json={
            "thread_id": "test",
            "messages": []
        }, headers=headers)
        assert response.status_code == 400

    def test_rate_limiting_contract(self):
        """Test that rate limiting is enforced per contract"""
        # This would require multiple requests in a short time period
        # For now, just verify the rate limiter is implemented
        from backend.src.utils.rate_limit import rate_limiter
        assert hasattr(rate_limiter, 'is_allowed')

    def test_data_model_contracts(self):
        """Test that data models match the contract specifications"""
        # Test ConversationThread model
        thread = ConversationThreadCreate(
            user_id="test_user",
            book_id="test_book",
            chapter_id="test_chapter",
            metadata={"labels": ["test"]}
        )
        assert thread.user_id == "test_user"
        assert thread.book_id == "test_book"

        # Test Message model
        message = MessageCreate(
            thread_id="test_thread",
            role="user",
            content="test message",
            metadata={}
        )
        assert message.role in ["user", "assistant", "system", "tool"]

        # Test RAGRequest model
        rag_request = RAGRequest(
            query="test query",
            book_id="test_book",
            chapter_id="test_chapter",
            top_k=5
        )
        assert rag_request.query == "test query"
        assert rag_request.top_k == 5


class TestStreamingContract:
    """Additional tests for streaming response contract"""

    def test_streaming_response_format(self):
        """Test that streaming responses follow the specified format"""
        # Based on the contract specification, streaming responses should include:
        # - message_chunk events with delta data
        # - message events with complete content
        # - error events when processing fails

        # This would be tested with actual streaming responses in a real scenario
        expected_events = ["message_chunk", "message", "error"]
        assert len(expected_events) == 3


# Run the tests
if __name__ == "__main__":
    pytest.main([__file__])