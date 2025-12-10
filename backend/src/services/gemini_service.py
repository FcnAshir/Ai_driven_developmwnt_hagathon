import os
import logging
from typing import AsyncGenerator, Dict, Any, Optional
import google.generativeai as genai
from google.generativeai.types import GenerateContentResponse
import asyncio

logger = logging.getLogger(__name__)

class GeminiService:
    """
    Service class to handle interactions with Google's Gemini API
    """

    def __init__(self):
        """
        Initialize the Gemini service with API configuration
        """
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise ValueError("GEMINI_API_KEY environment variable is required")

        genai.configure(api_key=api_key)
        model_name = os.getenv("GEMINI_MODEL", "gemini-pro")
        self.model = genai.GenerativeModel(model_name)
        self.temperature = float(os.getenv("LLM_TEMPERATURE", "0.7"))

    async def generate_stream(
        self,
        user_input: str,
        context: str,
        thread_id: str
    ) -> AsyncGenerator[str, None]:
        """
        Generate a streaming response from Gemini with the provided context and user input
        """
        logger.info(f"Generating stream response for thread {thread_id}, input length: {len(user_input)} chars")

        try:
            # Construct prompt with system message, context, and user query
            prompt = f"""
            SYSTEM: You are an expert assistant for the book. Use the context below to answer concisely and cite sources.

            CONTEXT:
            {context}

            USER QUESTION:
            {user_input}

            INSTRUCTIONS:
            1. Answer clearly and concisely.
            2. If uncertain, say so and provide sources from the context.
            3. Provide short summary + 1–3 action items when applicable.
            """

            # Generate response with streaming
            response = await self.model.generate_content_async(
                prompt,
                stream=True,
                generation_config=genai.types.GenerationConfig(
                    temperature=self.temperature
                )
            )

            logger.debug(f"Gemini response started for thread {thread_id}")

            # Stream the response chunks
            chunk_count = 0
            async for chunk in response:
                if chunk.text:
                    chunk_count += 1
                    logger.debug(f"Yielding chunk {chunk_count} for thread {thread_id}")
                    yield chunk.text

            logger.info(f"Successfully streamed {chunk_count} chunks for thread {thread_id}")

        except Exception as e:
            logger.error(f"Error generating response from Gemini for thread {thread_id}: {str(e)}", exc_info=True)
            # Return a helpful error message to the user
            yield f"I'm sorry, I encountered an error processing your request. Please try again later."
            # Re-raise the exception for upstream error handling if needed
            raise

    async def generate_response(
        self,
        user_input: str,
        context: str,
        thread_id: str
    ) -> str:
        """
        Generate a complete response from Gemini (non-streaming)
        """
        try:
            # Construct prompt with system message, context, and user query
            prompt = f"""
            SYSTEM: You are an expert assistant for the book. Use the context below to answer concisely and cite sources.

            CONTEXT:
            {context}

            USER QUESTION:
            {user_input}

            INSTRUCTIONS:
            1. Answer clearly and concisely.
            2. If uncertain, say so and provide sources from the context.
            3. Provide short summary + 1–3 action items when applicable.
            """

            response = await self.model.generate_content_async(prompt)
            return response.text if response.text else ""

        except Exception as e:
            logger.error(f"Error generating response from Gemini: {str(e)}")
            return "I'm sorry, I encountered an error processing your request. Please try again later."

    async def health_check(self) -> bool:
        """
        Check if the Gemini service is available and responding
        """
        try:
            # Test with a simple prompt
            response = await self.model.generate_content_async("Hello")
            return bool(response.text if response.text else "")
        except Exception as e:
            logger.error(f"Health check failed for Gemini service: {str(e)}")
            return False

    async def generate_response_with_navigation(
        self,
        user_input: str,
        context: str,
        thread_id: str,
        include_navigation: bool = True
    ) -> tuple[str, list]:
        """
        Generate response with potential navigation suggestions
        Returns both the response and a list of navigation suggestions
        """
        try:
            # Construct prompt with system message, context, and user query
            prompt = f"""
            SYSTEM: You are an expert assistant for the book. Use the context below to answer concisely and cite sources.
            When appropriate, suggest related sections the user might find useful.

            CONTEXT:
            {context}

            USER QUESTION:
            {user_input}

            INSTRUCTIONS:
            1. Answer clearly and concisely.
            2. If uncertain, say so and provide sources from the context.
            3. Provide short summary + 1–3 action items when applicable.
            4. IF the user's question relates to topics covered in other sections,
               suggest 1-2 relevant sections with brief explanations of why they're relevant.
            5. Format navigation suggestions as: "RELATED SECTIONS: [section reference] - [brief explanation]"
            """

            response = await self.model.generate_content_async(prompt)
            response_text = response.text if response.text else ""

            # Extract navigation suggestions if any
            navigation_suggestions = []
            if include_navigation and "RELATED SECTIONS:" in response_text:
                # Simple extraction - in a real system, this would be more sophisticated
                lines = response_text.split('\n')
                for line in lines:
                    if "RELATED SECTIONS:" in line:
                        # Extract the suggestion
                        suggestion = line.replace("RELATED SECTIONS:", "").strip()
                        navigation_suggestions.append(suggestion)

            return response_text, navigation_suggestions

        except Exception as e:
            logger.error(f"Error generating response with navigation from Gemini: {str(e)}")
            return "I'm sorry, I encountered an error processing your request. Please try again later.", []