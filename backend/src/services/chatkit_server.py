import logging
from typing import AsyncGenerator, Dict, Any, Optional
from fastapi import Depends
from ..models.conversation import ConversationThread
from ..models.message import Message, MessageCreate
from ..services.rag_service import RAGService
from ..services.gemini_service import GeminiService
from ..services.storage_service import SQLiteStorageService
from ..services.content_relationship_service import content_relationship_service, ContentRelationship

logger = logging.getLogger(__name__)

class ChatKitServer:
    """
    Main ChatKit server implementation that handles the ChatKit protocol
    and integrates with RAG and LLM services
    """

    def __init__(self):
        """
        Initialize the ChatKit server with required services
        """
        self.rag_service = RAGService()
        self.gemini_service = GeminiService()
        self.storage_service = SQLiteStorageService()

    async def get_conversation_context(self, thread_id: str) -> list:
        """
        Retrieve the conversation context (message history) for a given thread
        """
        try:
            # Get recent messages from the thread to provide context
            messages = await self.storage_service.get_thread_messages(thread_id)
            # Sort messages by creation time to maintain chronological order
            sorted_messages = sorted(messages, key=lambda x: x.created_at)

            # Format messages for context
            context_messages = []
            for msg in sorted_messages:
                context_messages.append({
                    "role": msg.role,
                    "content": msg.content
                })

            return context_messages
        except Exception as e:
            logger.warning(f"Could not retrieve conversation context: {str(e)}")
            return []

    async def respond(
        self,
        thread_id: str,
        user_input: str,
        context: Optional[Dict[str, Any]] = None
    ) -> AsyncGenerator[str, None]:
        """
        Main method to handle user input and generate streaming responses
        """
        try:
            # 1. Retrieve conversation context (message history)
            conversation_context = await self.get_conversation_context(thread_id)

            # 2. Retrieve relevant context from RAG
            rag_context = await self.rag_service.query(user_input, context)

            # 3. Combine conversation context with RAG context for the LLM
            combined_context = self._combine_contexts(conversation_context, rag_context)

            # 4. Generate response with navigation suggestions using Gemini
            response_text, navigation_suggestions = await self.gemini_service.generate_response_with_navigation(
                user_input, combined_context, thread_id
            )

            # 5. If there are navigation suggestions, add them to the response
            full_response = response_text
            if navigation_suggestions:
                # Add navigation suggestions to the response
                for suggestion in navigation_suggestions:
                    full_response += f"\n\nRELATED SECTIONS: {suggestion}"

            # 6. Stream the response
            response_parts = []
            # For streaming, we'll send the response as a whole since navigation suggestions
            # are part of the complete response
            for i, chunk in enumerate(full_response.split()):
                if i == 0:
                    yield chunk
                else:
                    yield f" {chunk}"
                response_parts.append(chunk)

            # 7. Save the user message and assistant response to storage
            try:
                # Save user message
                user_message = MessageCreate(
                    thread_id=thread_id,
                    role="user",
                    content=user_input
                )
                await self.storage_service.create_message(user_message)

                # Save assistant response
                assistant_response = full_response
                assistant_message = MessageCreate(
                    thread_id=thread_id,
                    role="assistant",
                    content=assistant_response
                )
                await self.storage_service.create_message(assistant_message)
            except Exception as e:
                logger.error(f"Error saving messages to storage: {str(e)}")
                # Don't raise the error as the response was already sent

        except Exception as e:
            logger.error(f"Error in ChatKit server respond method: {str(e)}")
            raise

    def _combine_contexts(self, conversation_context: list, rag_context: str) -> str:
        """
        Combine conversation history and RAG context into a single context string
        """
        # Format conversation history
        history_text = ""
        if conversation_context:
            history_text = "CONVERSATION HISTORY:\n"
            for msg in conversation_context[-5:]:  # Use last 5 messages as context
                history_text += f"{msg['role'].upper()}: {msg['content']}\n"
            history_text += "\n"

        # Combine with RAG context
        combined = f"{history_text}{rag_context}"
        return combined

    async def handle_action(
        self,
        action: str,
        payload: Dict[str, Any],
        thread_id: str
    ) -> Dict[str, Any]:
        """
        Handle custom actions from the ChatKit widget
        """
        try:
            if action == "search":
                query = payload.get("query", "")
                book_id = payload.get("book_id")
                chapter_id = payload.get("chapter_id")
                tags = payload.get("tags", [])
                top_k = payload.get("top_k", 5)

                # Perform advanced search with RAG service
                search_filters = {}
                if book_id:
                    search_filters["book_id"] = book_id
                if chapter_id:
                    search_filters["chapter_id"] = chapter_id
                if tags:
                    search_filters["tags"] = tags
                # Add option for duplicate removal
                search_filters["deduplicate"] = True

                search_results = await self.rag_service.advanced_search(
                    query=query,
                    filters=search_filters
                )

                # Format results for response
                formatted_results = [
                    {
                        "title": f"Section {i+1}",
                        "snippet": result.text[:200] + "..." if len(result.text) > 200 else result.text,
                        "source": result.source,
                        "score": result.score
                    }
                    for i, result in enumerate(search_results.results[:top_k])
                ]

                return {"results": formatted_results}

            elif action == "recommend":
                # Handle recommendation action
                pass
            elif action == "navigate_section":
                # Handle navigation action
                current_section = payload.get("current_section")
                topic = payload.get("topic")

                if current_section:
                    # Find related sections using the content relationship service
                    relationships = content_relationship_service.find_navigation_suggestions(
                        current_section, topic
                    )

                    navigation_suggestions = []
                    for rel in relationships:
                        navigation_suggestions.append({
                            "section": rel.to_section,
                            "type": rel.relationship_type,
                            "strength": rel.strength,
                            "reason": rel.reason
                        })

                    return {"navigation_suggestions": navigation_suggestions}
                else:
                    return {"navigation_suggestions": []}
            else:
                raise ValueError(f"Unknown action: {action}")

        except Exception as e:
            logger.error(f"Error handling action {action}: {str(e)}")
            raise