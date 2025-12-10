import pytest
from datetime import datetime
from backend.src.models.conversation import ConversationThread, ConversationThreadCreate, ConversationThreadUpdate
from backend.src.models.message import Message, MessageCreate, MessageUpdate
from backend.src.models.rag import RetrievedDocument, RAGRequest, RAGResponse


class TestConversationThread:
    """Unit tests for ConversationThread model and related models"""

    def test_conversation_thread_creation(self):
        """Test creating a ConversationThread instance"""
        thread = ConversationThread(
            thread_id="test_thread_id",
            user_id="test_user_id",
            book_id="test_book_id",
            chapter_id="test_chapter_id",
            created_at=datetime.now(),
            updated_at=datetime.now(),
            metadata={"test": "value"}
        )

        assert thread.thread_id == "test_thread_id"
        assert thread.user_id == "test_user_id"
        assert thread.book_id == "test_book_id"
        assert thread.chapter_id == "test_chapter_id"
        assert thread.metadata == {"test": "value"}

    def test_conversation_thread_create(self):
        """Test creating a ConversationThreadCreate instance"""
        thread_create = ConversationThreadCreate(
            user_id="test_user_id",
            book_id="test_book_id",
            chapter_id="test_chapter_id",
            metadata={"test": "value"}
        )

        assert thread_create.user_id == "test_user_id"
        assert thread_create.book_id == "test_book_id"
        assert thread_create.chapter_id == "test_chapter_id"
        assert thread_create.metadata == {"test": "value"}

    def test_conversation_thread_update(self):
        """Test creating a ConversationThreadUpdate instance"""
        thread_update = ConversationThreadUpdate(
            book_id="updated_book_id",
            chapter_id="updated_chapter_id",
            metadata={"updated": "value"}
        )

        assert thread_update.book_id == "updated_book_id"
        assert thread_update.chapter_id == "updated_chapter_id"
        assert thread_update.metadata == {"updated": "value"}


class TestMessage:
    """Unit tests for Message model and related models"""

    def test_message_creation(self):
        """Test creating a Message instance"""
        now = datetime.now()
        message = Message(
            message_id="test_message_id",
            thread_id="test_thread_id",
            role="user",
            content="test content",
            created_at=now,
            metadata={"test": "value"}
        )

        assert message.message_id == "test_message_id"
        assert message.thread_id == "test_thread_id"
        assert message.role == "user"
        assert message.content == "test content"
        assert message.created_at == now
        assert message.metadata == {"test": "value"}

    def test_message_create(self):
        """Test creating a MessageCreate instance"""
        message_create = MessageCreate(
            thread_id="test_thread_id",
            role="assistant",
            content="test content",
            metadata={"test": "value"}
        )

        assert message_create.thread_id == "test_thread_id"
        assert message_create.role == "assistant"
        assert message_create.content == "test content"
        assert message_create.metadata == {"test": "value"}

    def test_message_update(self):
        """Test creating a MessageUpdate instance"""
        message_update = MessageUpdate(
            content="updated content",
            metadata={"updated": "value"}
        )

        assert message_update.content == "updated content"
        assert message_update.metadata == {"updated": "value"}


class TestRetrievedDocument:
    """Unit tests for RetrievedDocument model and related models"""

    def test_retrieved_document_creation(self):
        """Test creating a RetrievedDocument instance"""
        doc = RetrievedDocument(
            document_id="test_doc_id",
            score=0.85,
            source="test:source",
            text="test document content",
            cursor=100
        )

        assert doc.document_id == "test_doc_id"
        assert doc.score == 0.85
        assert doc.source == "test:source"
        assert doc.text == "test document content"
        assert doc.cursor == 100

    def test_rag_request_creation(self):
        """Test creating a RAGRequest instance"""
        request = RAGRequest(
            query="test query",
            book_id="test_book",
            chapter_id="test_chapter",
            top_k=5
        )

        assert request.query == "test query"
        assert request.book_id == "test_book"
        assert request.chapter_id == "test_chapter"
        assert request.top_k == 5

    def test_rag_response_creation(self):
        """Test creating a RAGResponse instance"""
        doc = RetrievedDocument(
            document_id="test_doc_id",
            score=0.85,
            source="test:source",
            text="test document content",
            cursor=100
        )

        response = RAGResponse(
            query="test query",
            top_k=5,
            results=[doc],
            assembled_context="assembled context",
            retrieval_meta={"method": "qdrant"}
        )

        assert response.query == "test query"
        assert response.top_k == 5
        assert len(response.results) == 1
        assert response.results[0].document_id == "test_doc_id"
        assert response.assembled_context == "assembled context"
        assert response.retrieval_meta == {"method": "qdrant"}