from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from datetime import datetime

class RetrievedDocumentBase(BaseModel):
    document_id: str
    score: float  # Relevance score from the vector search (0.0 to 1.0)
    source: str   # Reference to the source (e.g., book:chapter:section)
    text: str     # The actual text content of the retrieved document
    cursor: Optional[int] = None  # Position marker in the original document

class RetrievedDocumentCreate(RetrievedDocumentBase):
    pass

class RetrievedDocumentUpdate(BaseModel):
    score: Optional[float] = None
    source: Optional[str] = None
    text: Optional[str] = None
    cursor: Optional[int] = None

class RetrievedDocument(RetrievedDocumentBase):
    class Config:
        from_attributes = True

class RAGResponse(BaseModel):
    query: str
    top_k: int
    results: List[RetrievedDocument]
    assembled_context: str
    retrieval_meta: Dict[str, Any] = {}

class RAGRequest(BaseModel):
    query: str
    book_id: Optional[str] = None
    chapter_id: Optional[str] = None
    top_k: int = 5