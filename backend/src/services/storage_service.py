from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any
from ..models.conversation import ConversationThread, ConversationThreadCreate, ConversationThreadUpdate
from ..models.message import Message, MessageCreate, MessageUpdate
from ..models.rag import RetrievedDocument
from datetime import datetime
import sqlite3
import json
import uuid
import logging

logger = logging.getLogger(__name__)

class StorageService(ABC):
    """
    Abstract base class for storage services
    """

    @abstractmethod
    async def create_thread(self, thread: ConversationThreadCreate) -> ConversationThread:
        pass

    @abstractmethod
    async def get_thread(self, thread_id: str) -> Optional[ConversationThread]:
        pass

    @abstractmethod
    async def update_thread(self, thread_id: str, thread_update: ConversationThreadUpdate) -> Optional[ConversationThread]:
        pass

    @abstractmethod
    async def create_message(self, message: MessageCreate) -> Message:
        pass

    @abstractmethod
    async def get_messages_by_thread(self, thread_id: str) -> List[Message]:
        pass

    @abstractmethod
    async def get_latest_messages(self, thread_id: str, limit: int = 10) -> List[Message]:
        pass


class SQLiteStorageService(StorageService):
    """
    SQLite implementation of the storage service
    """

    def __init__(self, db_path: str = "chat_threads.db"):
        self.db_path = db_path
        self.init_db()

    def init_db(self):
        """
        Initialize the database and create tables if they don't exist
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        # Create threads table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS threads (
                thread_id TEXT PRIMARY KEY,
                user_id TEXT NOT NULL,
                book_id TEXT,
                chapter_id TEXT,
                created_at TEXT NOT NULL,
                updated_at TEXT NOT NULL,
                metadata TEXT
            )
        """)

        # Create messages table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS messages (
                message_id TEXT PRIMARY KEY,
                thread_id TEXT NOT NULL,
                role TEXT NOT NULL,
                content TEXT NOT NULL,
                created_at TEXT NOT NULL,
                metadata TEXT,
                FOREIGN KEY (thread_id) REFERENCES threads (thread_id)
            )
        """)

        conn.commit()
        conn.close()

    async def create_thread(self, thread: ConversationThreadCreate) -> ConversationThread:
        logger.info(f"Creating new thread for user {thread.user_id}")
        thread_id = str(uuid.uuid4())
        created_at = datetime.utcnow()
        updated_at = created_at

        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        cursor.execute("""
            INSERT INTO threads (thread_id, user_id, book_id, chapter_id, created_at, updated_at, metadata)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, (
            thread_id,
            thread.user_id,
            thread.book_id,
            thread.chapter_id,
            created_at.isoformat(),
            updated_at.isoformat(),
            json.dumps(thread.metadata) if thread.metadata else None
        ))

        conn.commit()
        conn.close()

        logger.info(f"Thread {thread_id} created successfully for user {thread.user_id}")

        return ConversationThread(
            thread_id=thread_id,
            user_id=thread.user_id,
            book_id=thread.book_id,
            chapter_id=thread.chapter_id,
            created_at=created_at,
            updated_at=updated_at,
            metadata=thread.metadata
        )

    async def get_thread(self, thread_id: str) -> Optional[ConversationThread]:
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        cursor.execute("""
            SELECT thread_id, user_id, book_id, chapter_id, created_at, updated_at, metadata
            FROM threads
            WHERE thread_id = ?
        """, (thread_id,))

        row = cursor.fetchone()
        conn.close()

        if row is None:
            return None

        return ConversationThread(
            thread_id=row[0],
            user_id=row[1],
            book_id=row[2],
            chapter_id=row[3],
            created_at=datetime.fromisoformat(row[4]),
            updated_at=datetime.fromisoformat(row[5]),
            metadata=json.loads(row[6]) if row[6] else None
        )

    async def update_thread(self, thread_id: str, thread_update: ConversationThreadUpdate) -> Optional[ConversationThread]:
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        # Get current thread
        cursor.execute("""
            SELECT user_id, book_id, chapter_id, created_at, updated_at, metadata
            FROM threads
            WHERE thread_id = ?
        """, (thread_id,))

        row = cursor.fetchone()
        if row is None:
            conn.close()
            return None

        # Prepare update values
        updated_book_id = thread_update.book_id if thread_update.book_id is not None else row[1]
        updated_chapter_id = thread_update.chapter_id if thread_update.chapter_id is not None else row[2]
        updated_metadata = thread_update.metadata if thread_update.metadata is not None else json.loads(row[5]) if row[5] else None
        updated_at = datetime.utcnow()

        cursor.execute("""
            UPDATE threads
            SET book_id = ?, chapter_id = ?, updated_at = ?, metadata = ?
            WHERE thread_id = ?
        """, (
            updated_book_id,
            updated_chapter_id,
            updated_at.isoformat(),
            json.dumps(updated_metadata) if updated_metadata else None,
            thread_id
        ))

        conn.commit()
        conn.close()

        return ConversationThread(
            thread_id=thread_id,
            user_id=row[0],
            book_id=updated_book_id,
            chapter_id=updated_chapter_id,
            created_at=datetime.fromisoformat(row[3]),
            updated_at=updated_at,
            metadata=updated_metadata
        )

    async def create_message(self, message: MessageCreate) -> Message:
        logger.info(f"Creating message for thread {message.thread_id}, role: {message.role}")
        message_id = str(uuid.uuid4())
        created_at = datetime.utcnow()

        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        cursor.execute("""
            INSERT INTO messages (message_id, thread_id, role, content, created_at, metadata)
            VALUES (?, ?, ?, ?, ?, ?)
        """, (
            message_id,
            message.thread_id,
            message.role,
            message.content,
            created_at.isoformat(),
            json.dumps(message.metadata) if message.metadata else None
        ))

        conn.commit()
        conn.close()

        logger.debug(f"Message {message_id} created successfully in thread {message.thread_id}")

        return Message(
            message_id=message_id,
            thread_id=message.thread_id,
            role=message.role,
            content=message.content,
            created_at=created_at,
            metadata=message.metadata
        )

    async def get_messages_by_thread(self, thread_id: str) -> List[Message]:
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        cursor.execute("""
            SELECT message_id, thread_id, role, content, created_at, metadata
            FROM messages
            WHERE thread_id = ?
            ORDER BY created_at ASC
        """, (thread_id,))

        rows = cursor.fetchall()
        conn.close()

        messages = []
        for row in rows:
            messages.append(Message(
                message_id=row[0],
                thread_id=row[1],
                role=row[2],
                content=row[3],
                created_at=datetime.fromisoformat(row[4]),
                metadata=json.loads(row[5]) if row[5] else None
            ))

        return messages

    async def get_latest_messages(self, thread_id: str, limit: int = 10) -> List[Message]:
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        cursor.execute("""
            SELECT message_id, thread_id, role, content, created_at, metadata
            FROM messages
            WHERE thread_id = ?
            ORDER BY created_at DESC
            LIMIT ?
        """, (thread_id, limit))

        rows = cursor.fetchall()
        conn.close()

        messages = []
        for row in rows:
            messages.append(Message(
                message_id=row[0],
                thread_id=row[1],
                role=row[2],
                content=row[3],
                created_at=datetime.fromisoformat(row[4]),
                metadata=json.loads(row[5]) if row[5] else None
            ))

        # Reverse to return in chronological order
        return list(reversed(messages))

    # Alias for compatibility with ChatKit server
    async def get_thread_messages(self, thread_id: str) -> List[Message]:
        """
        Get all messages for a thread - alias for get_messages_by_thread
        """
        return await self.get_messages_by_thread(thread_id)


# Global instance
storage_service = SQLiteStorageService()