from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from datetime import datetime
from uuid import UUID, uuid4

class MessageContentItem(BaseModel):
    type: str  # "text", "image", etc.
    text: Optional[str] = None

class MessageBase(BaseModel):
    thread_id: str
    role: str  # "user", "assistant", "system", "tool"
    content: str
    metadata: Optional[Dict[str, Any]] = None

class MessageCreate(MessageBase):
    pass

class MessageUpdate(BaseModel):
    content: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None

class Message(MessageBase):
    message_id: str
    created_at: datetime

    class Config:
        from_attributes = True

class ChatMessage(BaseModel):
    id: str
    type: str  # "user|assistant|tool|system"
    content: List[MessageContentItem]
    created_at: datetime
    metadata: Dict[str, Any] = {}

    class Config:
        from_attributes = True