# Quickstart Guide: ChatKit + Gemini + RAG Integration

## Overview
This guide provides the essential information to get started with implementing the ChatKit integration with RAG and Gemini LLM backend.

## Prerequisites
- Python 3.11+
- Qdrant vector database instance
- Google Cloud account with Gemini API access
- JWT authentication system for the book frontend

## Setup Steps

### 1. Environment Configuration
```bash
# Copy the environment template
cp .env.example .env

# Update the following variables in .env:
GEMINI_API_KEY=your_gemini_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
JWT_SECRET=your_jwt_secret
DATABASE_URL=sqlite:///./chat_threads.db  # or PostgreSQL URL
```

### 2. Dependencies Installation
```bash
pip install fastapi uvicorn google-generativeai qdrant-client pydantic python-jose[cryptography] passlib[bcrypt]
```

### 3. Project Structure
```
backend/
├── src/
│   ├── models/
│   │   ├── conversation.py      # Conversation thread models
│   │   ├── message.py           # Message models
│   │   └── rag.py               # RAG result models
│   ├── services/
│   │   ├── chatkit_server.py    # ChatKit server implementation
│   │   ├── rag_service.py       # RAG pipeline integration
│   │   ├── gemini_service.py    # Gemini LLM adapter
│   │   └── storage_service.py   # Thread/message persistence
│   ├── api/
│   │   └── chatkit_endpoints.py # ChatKit protocol endpoints
│   └── main.py                  # Application entry point
```

### 4. Core Implementation

#### ChatKit Server Implementation
The main server class should implement the ChatKit protocol:

```python
from fastapi import FastAPI, Depends
from fastapi.responses import StreamingResponse
from typing import AsyncGenerator

class ChatKitServer:
    def __init__(self):
        self.rag_service = RAGService()
        self.gemini_service = GeminiService()
        self.storage_service = StorageService()

    async def respond(self, thread_id: str, user_input: str, context: dict) -> AsyncGenerator[str, None]:
        # 1. Retrieve relevant context from RAG
        rag_context = await self.rag_service.query(user_input, context)

        # 2. Generate response using Gemini with retrieved context
        async for chunk in self.gemini_service.generate_stream(
            user_input, rag_context, thread_id
        ):
            yield chunk
```

#### RAG Service Integration
The RAG service should connect to Qdrant and retrieve relevant book content:

```python
import qdrant_client
from qdrant_client.http import models

class RAGService:
    def __init__(self):
        self.client = qdrant_client.QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )

    async def query(self, query_text: str, context_filters: dict = None):
        # Create embedding for query
        query_embedding = await self.create_embedding(query_text)

        # Search in Qdrant with filters
        search_result = self.client.search(
            collection_name="book_content",
            query_vector=query_embedding,
            limit=5,
            filter=self.build_filters(context_filters) if context_filters else None
        )

        # Assemble context from results
        context_text = self.assemble_context(search_result)
        return context_text
```

#### Gemini Service Integration
The Gemini service should handle LLM calls with proper prompt engineering:

```python
import google.generativeai as genai

class GeminiService:
    def __init__(self):
        genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
        self.model = genai.GenerativeModel('gemini-pro')

    async def generate_stream(self, user_input: str, context: str, thread_id: str):
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
            stream=True
        )

        async for chunk in response:
            yield chunk.text
```

### 5. Running the Service
```bash
# Start the FastAPI server
uvicorn src.main:app --reload --port 8000

# The service will be available at:
# - POST /chatkit - Main ChatKit endpoint
# - GET /.well-known/health - Health check
```

### 6. Frontend Integration
The book frontend should embed the ChatKit widget with the correct API URL:

```html
<script src="https://cdn.chatkit.com/chatkit.js"></script>
<script>
  // Initialize ChatKit widget
  const chatkit = new ChatKit({
    apiURL: 'https://your-backend-domain/chatkit',
    fetch: (url, options) => {
      // Add JWT authentication header
      options.headers = {
        ...options.headers,
        'Authorization': `Bearer ${jwtToken}`
      };
      return fetch(url, options);
    }
  });
</script>
```

## Testing
Run the following to test your implementation:

```bash
# Unit tests
python -m pytest tests/unit/

# Integration tests
python -m pytest tests/integration/

# Contract tests
python -m pytest tests/contract/
```

## Key Configuration Parameters
- `TOP_K`: Number of documents to retrieve from RAG (default: 5)
- `MAX_CONTEXT_TOKENS`: Maximum tokens in context for LLM (default: 2500)
- `LLM_TEMPERATURE`: Gemini generation temperature (default: 0.7)
- `RATE_LIMIT_PER_MINUTE`: Requests per user per minute (default: 60)