import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from backend.src.services.content_relationship_service import (
    ContentRelationshipService,
    ContentRelationship,
    content_relationship_service
)
from backend.src.services.chatkit_server import ChatKitServer
from backend.src.services.gemini_service import GeminiService


class TestContentRelationshipService:
    """Unit tests for content relationship service"""

    def test_content_relationship_creation(self):
        """Test creating a content relationship object"""
        relationship = ContentRelationship(
            from_section="chapter_1",
            to_section="chapter_2",
            relationship_type="follows",
            strength=0.9,
            reason="Chapter 2 follows Chapter 1 sequentially"
        )

        assert relationship.from_section == "chapter_1"
        assert relationship.to_section == "chapter_2"
        assert relationship.relationship_type == "follows"
        assert relationship.strength == 0.9
        assert relationship.reason == "Chapter 2 follows Chapter 1 sequentially"

    def test_find_related_sections(self):
        """Test finding related sections"""
        service = ContentRelationshipService()

        # Add a test relationship
        test_rel = ContentRelationship(
            from_section="test_section",
            to_section="related_section",
            relationship_type="related_to",
            strength=0.8,
            reason="These sections are related"
        )
        service.add_relationship(test_rel)

        # Find related sections
        related = service.find_related_sections("test_section")

        assert len(related) == 1
        assert related[0].to_section == "related_section"
        assert related[0].strength == 0.8

    def test_find_related_sections_case_insensitive(self):
        """Test that finding related sections is case insensitive"""
        service = ContentRelationshipService()

        test_rel = ContentRelationship(
            from_section="MainSection",
            to_section="SubSection",
            relationship_type="expands_on",
            strength=0.7,
            reason="Subsection expands on main section"
        )
        service.add_relationship(test_rel)

        # Test with different cases
        related_upper = service.find_related_sections("MAINSECTION")
        related_lower = service.find_related_sections("mainsection")
        related_mixed = service.find_related_sections("MainSection")

        assert len(related_upper) == 1
        assert len(related_lower) == 1
        assert len(related_mixed) == 1

    def test_find_navigation_suggestions(self):
        """Test finding navigation suggestions"""
        service = ContentRelationshipService()

        test_rel = ContentRelationship(
            from_section="current",
            to_section="suggested",
            relationship_type="related_to",
            strength=0.85,
            reason="Suggested section is related to current"
        )
        service.add_relationship(test_rel)

        suggestions = service.find_navigation_suggestions("current")

        assert len(suggestions) == 1
        assert suggestions[0].to_section == "suggested"

    def test_relationship_limit(self):
        """Test that relationship finding respects the limit"""
        service = ContentRelationshipService()

        # Add multiple relationships
        for i in range(5):
            rel = ContentRelationship(
                from_section="test",
                to_section=f"section_{i}",
                relationship_type="related_to",
                strength=0.9 - (i * 0.1),  # Decreasing strength
                reason=f"Test relationship {i}"
            )
            service.add_relationship(rel)

        # Get only 3 results
        results = service.find_related_sections("test", limit=3)

        assert len(results) == 3
        # Should be sorted by strength (highest first)
        assert results[0].strength >= results[-1].strength

    def test_get_all_relationships(self):
        """Test getting all relationships"""
        service = ContentRelationshipService()
        initial_count = len(service.get_all_relationships())

        # Add a relationship
        new_rel = ContentRelationship(
            from_section="new_from",
            to_section="new_to",
            relationship_type="test_type",
            strength=0.5,
            reason="Test relationship"
        )
        service.add_relationship(new_rel)

        all_rels = service.get_all_relationships()
        assert len(all_rels) == initial_count + 1


class TestNavigationWithGemini:
    """Tests for navigation functionality with Gemini integration"""

    @pytest.mark.asyncio
    async def test_generate_response_with_navigation(self):
        """Test generating response with navigation suggestions"""
        gemini_service = GeminiService()

        # Mock the model response to include navigation suggestions
        with patch.object(gemini_service, 'model') as mock_model:
            mock_response = MagicMock()
            mock_response.text = "This is the answer to your question.\n\nRELATED SECTIONS: Chapter 3 - This expands on the concepts mentioned here"
            mock_model.generate_content_async.return_value = mock_response

            response, suggestions = await gemini_service.generate_response_with_navigation(
                "test question",
                "test context",
                "test_thread"
            )

            assert "This is the answer" in response
            assert len(suggestions) == 1
            assert "Chapter 3" in suggestions[0]

    @pytest.mark.asyncio
    async def test_generate_response_with_navigation_error_handling(self):
        """Test error handling in navigation response generation"""
        gemini_service = GeminiService()

        with patch.object(gemini_service, 'model.generate_content_async', side_effect=Exception("API Error")):
            response, suggestions = await gemini_service.generate_response_with_navigation(
                "test question",
                "test context",
                "test_thread"
            )

            # Should return error message and empty suggestions
            assert "error processing your request" in response
            assert len(suggestions) == 0


class TestChatKitNavigation:
    """Tests for ChatKit server navigation features"""

    @pytest.mark.asyncio
    async def test_handle_navigate_section_action(self):
        """Test handling the navigate section action"""
        server = ChatKitServer()

        # Mock the content relationship service
        with patch('backend.src.services.chatkit_server.content_relationship_service') as mock_service:
            mock_rel = MagicMock()
            mock_rel.to_section = "chapter_3"
            mock_rel.relationship_type = "related_to"
            mock_rel.strength = 0.8
            mock_rel.reason = "Related concept"

            mock_service.find_navigation_suggestions.return_value = [mock_rel]

            result = await server.handle_action(
                "navigate_section",
                {"current_section": "chapter_1", "topic": "algorithms"},
                "test_thread"
            )

            assert "navigation_suggestions" in result
            assert len(result["navigation_suggestions"]) == 1
            assert result["navigation_suggestions"][0]["section"] == "chapter_3"

    @pytest.mark.asyncio
    async def test_handle_navigate_section_no_current_section(self):
        """Test navigate section action with no current section"""
        server = ChatKitServer()

        result = await server.handle_action(
            "navigate_section",
            {"topic": "algorithms"},  # No current_section
            "test_thread"
        )

        assert "navigation_suggestions" in result
        assert len(result["navigation_suggestions"]) == 0