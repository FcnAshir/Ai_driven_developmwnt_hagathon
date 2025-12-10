import re
from typing import Any, Optional
from pydantic import BaseModel, validator, ValidationError
from fastapi import HTTPException, status
import html
import bleach


class InputValidator:
    """
    Utility class for input validation and sanitization
    """

    @staticmethod
    def validate_thread_id(thread_id: str) -> str:
        """
        Validate thread ID format
        """
        if not thread_id:
            raise ValueError("Thread ID cannot be empty")

        # Check if thread_id is a valid UUID or a properly formatted string
        if not re.match(r'^[a-zA-Z0-9_-]+$', thread_id):
            raise ValueError("Invalid thread ID format")

        return thread_id

    @staticmethod
    def validate_user_id(user_id: str) -> str:
        """
        Validate user ID format
        """
        if not user_id:
            raise ValueError("User ID cannot be empty")

        if not re.match(r'^[a-zA-Z0-9_-]+$', user_id):
            raise ValueError("Invalid user ID format")

        return user_id

    @staticmethod
    def validate_content(content: str) -> str:
        """
        Validate and sanitize content
        """
        if not content or not content.strip():
            raise ValueError("Content cannot be empty")

        # Check length
        if len(content) > 10000:  # 10k character limit
            raise ValueError("Content exceeds maximum length of 10,000 characters")

        # Sanitize HTML content
        sanitized = InputValidator.sanitize_text(content)

        return sanitized

    @staticmethod
    def validate_role(role: str) -> str:
        """
        Validate message role
        """
        valid_roles = {"user", "assistant", "system", "tool"}
        if role not in valid_roles:
            raise ValueError(f"Invalid role. Must be one of: {', '.join(valid_roles)}")

        return role

    @staticmethod
    def sanitize_text(text: str) -> str:
        """
        Sanitize text input to prevent XSS and other injection attacks
        """
        if not text:
            return text

        # First, use bleach to strip potentially dangerous HTML tags
        # Only allow safe tags like <em>, <strong>, etc., if needed
        # For now, we'll strip all HTML tags
        sanitized = bleach.clean(text, strip=True, tags=[])

        # Escape HTML entities
        sanitized = html.escape(sanitized)

        # Remove any remaining dangerous patterns
        dangerous_patterns = [
            r'<script.*?>.*?</script>',  # Script tags
            r'javascript:',              # JavaScript URLs
            r'vbscript:',               # VBScript URLs
            r'on\w+\s*=',               # Event handlers
        ]

        for pattern in dangerous_patterns:
            sanitized = re.sub(pattern, '', sanitized, flags=re.IGNORECASE)

        return sanitized.strip()

    @staticmethod
    def validate_book_id(book_id: Optional[str]) -> Optional[str]:
        """
        Validate book ID if provided
        """
        if not book_id:
            return book_id

        if not re.match(r'^[a-zA-Z0-9_-]+$', book_id):
            raise ValueError("Invalid book ID format")

        return book_id

    @staticmethod
    def validate_chapter_id(chapter_id: Optional[str]) -> Optional[str]:
        """
        Validate chapter ID if provided
        """
        if not chapter_id:
            return chapter_id

        if not re.match(r'^[a-zA-Z0-9_-]+$', chapter_id):
            raise ValueError("Invalid chapter ID format")

        return chapter_id

    @staticmethod
    def validate_query(query: str) -> str:
        """
        Validate search query
        """
        if not query or not query.strip():
            raise ValueError("Query cannot be empty")

        query = query.strip()

        # Check length
        if len(query) > 1000:
            raise ValueError("Query exceeds maximum length of 1,000 characters")

        # Sanitize
        sanitized = InputValidator.sanitize_text(query)

        return sanitized


class ThreadValidationModel(BaseModel):
    """
    Pydantic model for validating thread data
    """
    user_id: str
    book_id: Optional[str] = None
    chapter_id: Optional[str] = None
    metadata: Optional[dict] = None

    @validator('user_id')
    def validate_user_id(cls, v):
        return InputValidator.validate_user_id(v)

    @validator('book_id', 'chapter_id', pre=True)
    def validate_optional_ids(cls, v):
        if v is not None:
            # Use the appropriate validator based on field name
            if cls.__name__ == 'ThreadValidationModel' and 'book_id' in cls.__fields__:
                return InputValidator.validate_book_id(v)
            elif cls.__name__ == 'ThreadValidationModel' and 'chapter_id' in cls.__fields__:
                return InputValidator.validate_chapter_id(v)
        return v


class MessageValidationModel(BaseModel):
    """
    Pydantic model for validating message data
    """
    thread_id: str
    role: str
    content: str
    metadata: Optional[dict] = None

    @validator('thread_id')
    def validate_thread_id(cls, v):
        return InputValidator.validate_thread_id(v)

    @validator('role')
    def validate_role(cls, v):
        return InputValidator.validate_role(v)

    @validator('content')
    def validate_content(cls, v):
        return InputValidator.validate_content(v)


def validate_and_sanitize_input(data_type: str, data: Any) -> Any:
    """
    General function to validate and sanitize input based on type
    """
    try:
        if data_type == "thread":
            # Validate thread data
            validated = ThreadValidationModel(**data)
            return validated.dict()
        elif data_type == "message":
            # Validate message data
            validated = MessageValidationModel(**data)
            return validated.dict()
        else:
            raise ValueError(f"Unknown data type: {data_type}")
    except ValidationError as e:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail=f"Validation error: {e.json()}"
        )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail=str(e)
        )


def sanitize_user_input(text: str) -> str:
    """
    Convenience function to sanitize user input
    """
    return InputValidator.sanitize_text(text)