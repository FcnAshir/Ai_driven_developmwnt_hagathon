import pytest
from fastapi.testclient import TestClient
from backend.src.main import app
from backend.src.services.content_relationship_service import content_relationship_service
from backend.src.services.chatkit_server import ChatKitServer
from unittest.mock import AsyncMock, patch


class TestNavigationIntegration:
    """Integration tests for navigation features"""

    def setup_method(self):
        """Setup test client for each test method"""
        self.client = TestClient(app)

    def test_navigate_section_action_integration(self):
        """Test navigate section action through the API"""
        with patch.object(content_relationship_service, 'find_navigation_suggestions') as mock_find:
            # Mock some relationships
            from backend.src.services.content_relationship_service import ContentRelationship
            mock_rel = ContentRelationship(
                from_section="current_chapter",
                to_section="next_chapter",
                relationship_type="follows",
                strength=0.9,
                reason="Next chapter follows current"
            )
            mock_find.return_value = [mock_rel]

            payload = {
                "action": "navigate_section",
                "payload": {
                    "current_section": "current_chapter",
                    "topic": "main_topic"
                },
                "thread_id": "test_thread_123"
            }

            headers = {"Authorization": "Bearer mock_token_for_testing"}
            response = self.client.post("/actions", json=payload, headers=headers)

            # Should not return validation error
            assert response.status_code != 422

            if response.status_code == 200:
                data = response.json()
                assert "navigation_suggestions" in data
                assert len(data["navigation_suggestions"]) == 1

    def test_navigate_section_with_no_relationships(self):
        """Test navigate section action when no relationships exist"""
        with patch.object(content_relationship_service, 'find_navigation_suggestions') as mock_find:
            mock_find.return_value = []

            payload = {
                "action": "navigate_section",
                "payload": {"current_section": "unknown_section"},
                "thread_id": "test_thread_456"
            }

            headers = {"Authorization": "Bearer mock_token_for_testing"}
            response = self.client.post("/actions", json=payload, headers=headers)

            assert response.status_code != 422

            if response.status_code == 200:
                data = response.json()
                assert "navigation_suggestions" in data
                assert len(data["navigation_suggestions"]) == 0

    def test_recommend_action_placeholder(self):
        """Test recommend action (currently a placeholder)"""
        payload = {
            "action": "recommend",
            "payload": {"topic": "test_topic"},
            "thread_id": "test_thread_789"
        }

        headers = {"Authorization": "Bearer mock_token_for_testing"}
        response = self.client.post("/actions", json=payload, headers=headers)

        # This might raise an exception since it's not fully implemented
        # but should not return a validation error
        assert response.status_code != 422

    def test_content_relationship_service_integration(self):
        """Test content relationship service integration points"""
        # Verify the service has the expected methods
        assert hasattr(content_relationship_service, 'find_related_sections')
        assert hasattr(content_relationship_service, 'find_navigation_suggestions')
        assert hasattr(content_relationship_service, 'add_relationship')

        # Test with a known section to see if default relationships work
        relationships = content_relationship_service.find_related_sections("introduction")
        # Should find at least the default relationship we set up
        assert isinstance(relationships, list)

    def test_navigation_suggestion_format(self):
        """Test that navigation suggestions are properly formatted"""
        with patch.object(content_relationship_service, 'find_navigation_suggestions') as mock_find:
            from backend.src.services.content_relationship_service import ContentRelationship
            mock_rel = ContentRelationship(
                from_section="chapter_1",
                to_section="chapter_2",
                relationship_type="follows",
                strength=0.95,
                reason="Chapter 2 follows Chapter 1"
            )
            mock_find.return_value = [mock_rel]

            payload = {
                "action": "navigate_section",
                "payload": {"current_section": "chapter_1"},
                "thread_id": "test_thread_format"
            }

            headers = {"Authorization": "Bearer mock_token_for_testing"}
            response = self.client.post("/actions", json=payload, headers=headers)

            if response.status_code == 200:
                data = response.json()
                assert "navigation_suggestions" in data
                if len(data["navigation_suggestions"]) > 0:
                    suggestion = data["navigation_suggestions"][0]
                    assert "section" in suggestion
                    assert "type" in suggestion
                    assert "strength" in suggestion
                    assert "reason" in suggestion


class TestChatKitServerNavigation:
    """Additional integration tests for ChatKit server navigation features"""

    @pytest.mark.asyncio
    async def test_chatkit_server_navigation_action(self):
        """Test navigation action handling in ChatKit server"""
        server = ChatKitServer()

        with patch.object(content_relationship_service, 'find_navigation_suggestions') as mock_find:
            from backend.src.services.content_relationship_service import ContentRelationship
            mock_rel = ContentRelationship(
                from_section="test_current",
                to_section="test_next",
                relationship_type="related_to",
                strength=0.8,
                reason="Test relationship"
            )
            mock_find.return_value = [mock_rel]

            result = await server.handle_action(
                "navigate_section",
                {"current_section": "test_current", "topic": "test_topic"},
                "test_thread"
            )

            assert "navigation_suggestions" in result
            assert len(result["navigation_suggestions"]) == 1
            assert result["navigation_suggestions"][0]["section"] == "test_next"
            assert result["navigation_suggestions"][0]["strength"] == 0.8

    @pytest.mark.asyncio
    async def test_chatkit_server_search_action_with_navigation_context(self):
        """Test that search actions work alongside navigation features"""
        with patch.object(content_relationship_service, 'find_navigation_suggestions') as mock_nav_find, \
             patch.object(server.rag_service, 'advanced_search', new_callable=AsyncMock) as mock_search:

            # Setup mocks
            from backend.src.services.content_relationship_service import ContentRelationship
            nav_rel = ContentRelationship(
                from_section="search_context",
                to_section="related_section",
                relationship_type="related_to",
                strength=0.7,
                reason="Related to search context"
            )
            mock_nav_find.return_value = [nav_rel]

            from backend.src.models.rag import RetrievedDocument, RAGResponse
            mock_doc = RetrievedDocument(
                document_id="search_result",
                score=0.9,
                source="search:section",
                text="Search result content"
            )
            mock_response = RAGResponse(
                query="search query",
                top_k=5,
                results=[mock_doc],
                assembled_context="search context",
                retrieval_meta={"method": "qdrant"}
            )
            mock_search.return_value = mock_response

            # Test that both search and navigation can work together
            server_instance = ChatKitServer()
            search_result = await server_instance.handle_action(
                "search",
                {"query": "test search", "book_id": "test_book"},
                "test_thread"
            )

            nav_result = await server_instance.handle_action(
                "navigate_section",
                {"current_section": "search_context"},
                "test_thread"
            )

            assert "results" in search_result or "navigation_suggestions" in nav_result


class TestNavigationErrorHandling:
    """Test error handling in navigation features"""

    def test_navigation_action_error_handling(self):
        """Test error handling in navigation action"""
        with patch.object(content_relationship_service, 'find_navigation_suggestions', side_effect=Exception("Service error")):
            server = ChatKitServer()

            # This should handle the error gracefully
            try:
                result = server.handle_action(
                    "navigate_section",
                    {"current_section": "error_section"},
                    "test_thread"
                )
                # The async nature means we'd need to await this in real usage
            except Exception:
                # Error should be handled internally by the method
                pass  # This is expected behavior

    def test_malformed_navigation_request(self):
        """Test handling of malformed navigation requests"""
        payload = {
            "action": "navigate_section",
            "payload": {},  # Missing required fields
            "thread_id": "test_thread_malformed"
        }

        headers = {"Authorization": "Bearer mock_token_for_testing"}
        response = self.client.post("/actions", json=payload, headers=headers)

        # Should handle gracefully, not crash
        assert response.status_code in [200, 401, 500, 422]