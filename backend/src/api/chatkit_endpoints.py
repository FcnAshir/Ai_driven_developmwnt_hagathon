from fastapi import APIRouter, HTTPException, Depends, Request
from fastapi.responses import JSONResponse, StreamingResponse
import logging
from datetime import datetime
from typing import Optional
import os
import json
import re
from pydantic import BaseModel
from ..services.chatkit_server import ChatKitServer
from ..utils.auth import get_current_user
from ..utils.rate_limit import rate_limit_middleware
from ..utils.validation import InputValidator, validate_and_sanitize_input

router = APIRouter()
logger = logging.getLogger(__name__)

# Initialize ChatKit server
chatkit_server = ChatKitServer()

class ChatKitRequest(BaseModel):
    thread_id: str
    messages: list
    context: Optional[dict] = None

@router.get("/.well-known/health")
async def health_check():
    """
    Health check endpoint that returns the status of the service
    """
    try:
        # Add any additional health checks here (database connectivity, etc.)
        return {
            "status": "healthy",
            "timestamp": datetime.utcnow().isoformat(),
            "version": os.getenv("APP_VERSION", "0.1.0"),
            "service": "chatkit-gemini-rag-integration"
        }
    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        return JSONResponse(
            status_code=503,
            content={
                "status": "unhealthy",
                "timestamp": datetime.utcnow().isoformat(),
                "error": str(e)
            }
        )

@router.post("/chatkit")
async def chatkit_endpoint(
    request: ChatKitRequest,
    current_user: dict = Depends(get_current_user)
):
    """
    Main ChatKit endpoint that handles user messages and returns streaming responses
    """
    try:
        # Apply rate limiting
        rate_limit_middleware(current_user["user_id"])

        # Validate thread_id
        validated_thread_id = InputValidator.validate_thread_id(request.thread_id)

        # Extract the latest user message from the messages list
        if not request.messages:
            raise HTTPException(status_code=400, detail="No messages provided")

        # Get the last message which should be the user's input
        user_message = request.messages[-1]
        if user_message.get("role") != "user":
            raise HTTPException(status_code=400, detail="Last message must be from user")

        # Validate and sanitize user input
        user_input = user_message.get("content", "")
        if not user_input:
            raise HTTPException(status_code=400, detail="User message content cannot be empty")

        sanitized_input = InputValidator.validate_content(user_input)

        # Sanitize context if provided
        sanitized_context = None
        if request.context:
            sanitized_context = {}
            if request.context.get("book_id"):
                sanitized_context["book_id"] = InputValidator.validate_book_id(request.context["book_id"])
            if request.context.get("chapter_id"):
                sanitized_context["chapter_id"] = InputValidator.validate_chapter_id(request.context["chapter_id"])

        # Process the request using the ChatKit server
        async def event_generator():
            try:
                async for chunk in chatkit_server.respond(
                    thread_id=validated_thread_id,
                    user_input=sanitized_input,
                    context=sanitized_context
                ):
                    # Yield message chunk event
                    yield f"event: message_chunk\n"
                    yield f"data: {json.dumps({'delta': chunk})}\n\n"

                # Send completion event
                yield f"event: message\n"
                yield f"data: {json.dumps({'content': 'Response completed', 'role': 'assistant'})}\n\n"
            except Exception as e:
                logger.error(f"Error in streaming response: {str(e)}")
                yield f"event: error\n"
                yield f"data: {json.dumps({'error': str(e)})}\n\n"

        return StreamingResponse(
            event_generator(),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "Access-Control-Allow-Origin": "*",
            }
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in chatkit endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.post("/chatkit-public")
async def chatkit_public_endpoint(request: Request):
    """
    Public ChatKit endpoint that handles user messages without authentication for frontend integration
    """
    try:
        # Apply rate limiting using IP address as identifier
        # Get client IP from request
        client_ip = request.headers.get("x-forwarded-for", request.client.host)
        rate_limit_middleware(client_ip)

        # Parse the JSON body
        json_data = await request.json()
        chat_request = ChatKitRequest(**json_data)

        # Validate thread_id
        validated_thread_id = InputValidator.validate_thread_id(chat_request.thread_id)

        # Extract the latest user message from the messages list
        if not chat_request.messages:
            raise HTTPException(status_code=400, detail="No messages provided")

        # Get the last message which should be the user's input
        user_message = chat_request.messages[-1]
        if user_message.get("role") != "user":
            raise HTTPException(status_code=400, detail="Last message must be from user")

        # Validate and sanitize user input
        user_input = user_message.get("content", "")
        if not user_input:
            raise HTTPException(status_code=400, detail="User message content cannot be empty")

        sanitized_input = InputValidator.validate_content(user_input)

        # Sanitize context if provided
        sanitized_context = None
        if chat_request.context:
            sanitized_context = {}
            if chat_request.context.get("book_id"):
                sanitized_context["book_id"] = InputValidator.validate_book_id(chat_request.context["book_id"])
            if chat_request.context.get("chapter_id"):
                sanitized_context["chapter_id"] = InputValidator.validate_chapter_id(chat_request.context["chapter_id"])

        # Process the request using the ChatKit server
        async def event_generator():
            try:
                async for chunk in chatkit_server.respond(
                    thread_id=validated_thread_id,
                    user_input=sanitized_input,
                    context=sanitized_context
                ):
                    # Yield message chunk event
                    yield f"event: message_chunk\n"
                    yield f"data: {json.dumps({'delta': chunk})}\n\n"

                # Send completion event
                yield f"event: message\n"
                yield f"data: {json.dumps({'content': 'Response completed', 'role': 'assistant'})}\n\n"
            except Exception as e:
                logger.error(f"Error in streaming response: {str(e)}")
                yield f"event: error\n"
                yield f"data: {json.dumps({'error': str(e)})}\n\n"

        return StreamingResponse(
            event_generator(),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "Access-Control-Allow-Origin": "*",
            }
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in chatkit public endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")

@router.post("/actions")
async def actions_endpoint(
    request: Request,
    current_user: dict = Depends(get_current_user)
):
    """
    Endpoint for handling custom actions from the ChatKit widget
    """
    try:
        # Apply rate limiting
        rate_limit_middleware(current_user["user_id"])

        payload = await request.json()
        action = payload.get("action")
        action_payload = payload.get("payload", {})
        thread_id = payload.get("thread_id")

        if not action or not thread_id:
            raise HTTPException(status_code=400, detail="Action and thread_id are required")

        # Validate thread_id
        validated_thread_id = InputValidator.validate_thread_id(thread_id)

        # Validate action
        if not re.match(r'^[a-zA-Z0-9_-]+$', str(action)):
            raise HTTPException(status_code=400, detail="Invalid action format")

        # Sanitize action payload content
        sanitized_payload = {}
        for key, value in action_payload.items():
            if isinstance(value, str):
                sanitized_payload[key] = InputValidator.sanitize_text(value)
            else:
                sanitized_payload[key] = value

        # Handle the action using the ChatKit server
        result = await chatkit_server.handle_action(action, sanitized_payload, validated_thread_id)
        return result

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in actions endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")