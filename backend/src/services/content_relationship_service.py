import logging
from typing import List, Dict, Any, Optional
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class ContentRelationship:
    """
    Represents a relationship between content sections
    """
    from_section: str
    to_section: str
    relationship_type: str  # "related_to", "follows", "prerequisite", "expands_on"
    strength: float  # 0.0 to 1.0, higher means stronger relationship
    reason: str  # Explanation of the relationship


class ContentRelationshipService:
    """
    Service for managing content relationships and navigation suggestions
    """

    def __init__(self):
        # In a real implementation, this would load from a database or file
        # For now, we'll use an in-memory store
        self.relationships: List[ContentRelationship] = []
        self._load_default_relationships()

    def _load_default_relationships(self):
        """
        Load default content relationships for the book
        In a real implementation, this would load from a persisted store
        """
        # Example relationships - these would be populated based on the actual book content
        default_relationships = [
            ContentRelationship(
                from_section="introduction",
                to_section="chapter_1",
                relationship_type="follows",
                strength=1.0,
                reason="Chapter 1 follows the introduction"
            ),
            ContentRelationship(
                from_section="chapter_1",
                to_section="chapter_2",
                relationship_type="follows",
                strength=1.0,
                reason="Chapter 2 follows Chapter 1 sequentially"
            ),
            ContentRelationship(
                from_section="chapter_2",
                to_section="chapter_1",
                relationship_type="related_to",
                strength=0.7,
                reason="Chapter 2 builds on concepts introduced in Chapter 1"
            ),
            ContentRelationship(
                from_section="algorithms",
                to_section="data_structures",
                relationship_type="prerequisite",
                strength=0.9,
                reason="Understanding data structures is important before learning algorithms"
            ),
            ContentRelationship(
                from_section="data_structures",
                to_section="algorithms",
                relationship_type="related_to",
                strength=0.8,
                reason="Algorithms often use data structures"
            ),
        ]
        self.relationships.extend(default_relationships)

    def find_related_sections(self, current_section: str, limit: int = 3) -> List[ContentRelationship]:
        """
        Find sections related to the current section
        """
        related = []
        for rel in self.relationships:
            if rel.from_section.lower() == current_section.lower():
                related.append(rel)

        # Sort by relationship strength
        related.sort(key=lambda x: x.strength, reverse=True)
        return related[:limit]

    def find_navigation_suggestions(self, current_section: str, topic: str = None) -> List[ContentRelationship]:
        """
        Find navigation suggestions based on current section and optional topic
        """
        suggestions = []

        # Find direct relationships
        direct_rels = self.find_related_sections(current_section)
        suggestions.extend(direct_rels)

        # In a real implementation, we might also search by topic/content similarity
        # For now, return the direct relationships
        return suggestions

    def add_relationship(self, relationship: ContentRelationship):
        """
        Add a new content relationship
        """
        self.relationships.append(relationship)

    def get_all_relationships(self) -> List[ContentRelationship]:
        """
        Get all content relationships
        """
        return self.relationships.copy()


# Global instance
content_relationship_service = ContentRelationshipService()