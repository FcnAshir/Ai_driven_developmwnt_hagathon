# Data Model: ChatKit + Gemini + RAG Integration

## Entity: Conversation Thread
**Description**: Represents a user's ongoing conversation with the chatbot, maintaining context across multiple exchanges

**Fields**:
- `thread_id`: UUID (Primary Key) - Unique identifier for the conversation thread
- `user_id`: String - Identifier for the user who owns this thread
- `book_id`: String (Optional) - Identifier for the book associated with this conversation
- `chapter_id`: String (Optional) - Identifier for the specific chapter if context is chapter-specific
- `created_at`: ISO8601 DateTime - Timestamp when the thread was created
- `updated_at`: ISO8601 DateTime - Timestamp when the thread was last updated
- `metadata`: JSON Object - Additional metadata for the thread (labels, last response run ID, etc.)

**Validation Rules**:
- `thread_id` must be unique across all threads
- `user_id` must be present
- `created_at` is auto-generated on creation
- `updated_at` is auto-updated on any modification

**State Transitions**:
- Active: Thread is open for new messages
- Archived: Thread is read-only after period of inactivity

## Entity: Chat Message
**Description**: Represents a single message in the conversation, containing user input or system response

**Fields**:
- `message_id`: UUID (Primary Key) - Unique identifier for the message
- `thread_id`: UUID (Foreign Key) - Reference to the conversation thread
- `role`: String (user|assistant|system|tool) - Role of the message sender
- `content`: String - The actual message content
- `created_at`: ISO8601 DateTime - Timestamp when the message was created
- `metadata`: JSON Object - Additional metadata (source attribution, action payloads, etc.)

**Validation Rules**:
- `thread_id` must reference an existing conversation thread
- `role` must be one of the allowed values
- `content` must not be empty
- `created_at` is auto-generated on creation

## Entity: Retrieved Document
**Description**: Represents book content retrieved from Qdrant that is relevant to the user's query

**Fields**:
- `document_id`: String (Primary Key) - Unique identifier for the document
- `score`: Float - Relevance score from the vector search (0.0 to 1.0)
- `source`: String - Reference to the source (e.g., book:chapter:section)
- `text`: String - The actual text content of the retrieved document
- `cursor`: Integer (Optional) - Position marker in the original document

**Validation Rules**:
- `document_id` must be unique
- `score` must be between 0.0 and 1.0
- `text` must not be empty
- `source` must be a valid reference format

## Relationships:
- Conversation Thread (1) ←→ (Many) Chat Message
- Conversation Thread (1) ←→ (Many) Retrieved Document (during a specific query)

## State Management:
- Conversation threads maintain their context across multiple messages
- Retrieved documents are associated with specific queries within threads
- Thread metadata is updated as new messages are added