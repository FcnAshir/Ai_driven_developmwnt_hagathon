from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from datetime import datetime
from uuid import UUID, uuid4

class ConversationThreadBase(BaseModel):
    user_id: str
    book_id: Optional[str] = None
    chapter_id: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None

class ConversationThreadCreate(ConversationThreadBase):
    pass

class ConversationThreadUpdate(BaseModel):
    book_id: Optional[str] = None
    chapter_id: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None

class ConversationThread(ConversationThreadBase):
    thread_id: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class ThreadMetadata(BaseModel):
    thread_id: str
    user_id: str
    book_id: Optional[str] = None
    chapter_id: Optional[str] = None
    last_response_run_id: Optional[str] = None
    labels: List[str] = []