# AI-Driven Development Hackathon - ChatKit + Gemini + RAG Integration

This project implements a ChatKit-based chatbot integrated with a FastAPI backend, Google's Gemini LLM, and a Retrieval-Augmented Generation (RAG) system for a digital book. The system provides contextual responses based on book content stored in Qdrant, with streaming responses, thread context maintenance, and custom actions.

## Features

- Interactive chat with book content using natural language
- Enhanced search via RAG (Retrieval-Augmented Generation)
- Actionable responses with navigation suggestions
- Thread persistence and conversation context management
- Streaming responses for real-time feedback
- Rate limiting and security features
- JWT-based authentication

## Architecture

The system consists of:

- **FastAPI Backend**: Handles API requests and orchestrates the services
- **Qdrant Vector Database**: Stores and retrieves book content for RAG
- **Google Gemini**: LLM for generating responses
- **Storage Service**: Persists conversation threads and messages
- **ChatKit Integration**: Frontend widget for user interaction

## Prerequisites

- Python 3.11+
- Qdrant vector database instance
- Google Cloud account with Gemini API access
- JWT authentication system for the book frontend

## Setup

1. Clone the repository
2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```
3. Install dependencies:
   ```bash
   pip install -r backend/requirements.txt
   ```
4. Configure environment variables in `backend/.env`:
   ```bash
   cp backend/.env.example backend/.env
   # Update the variables in backend/.env with your values
   ```
5. Start the application:
   ```bash
   cd backend
   uvicorn src.main:app --reload
   ```

## Environment Variables

- `GEMINI_API_KEY`: Your Google Gemini API key
- `QDRANT_URL`: URL for your Qdrant instance
- `QDRANT_API_KEY`: API key for Qdrant
- `JWT_SECRET`: Secret key for JWT token signing
- `DATABASE_URL`: Connection string for the database
- `RATE_LIMIT_PER_MINUTE`: Rate limit per user per minute

## API Endpoints

- `GET /.well-known/health`: Health check endpoint
- `POST /chatkit`: Main ChatKit endpoint for chat interactions
- `POST /actions`: Endpoint for custom actions (search, navigation, etc.)

## Security Features

- JWT token authentication
- Input validation and sanitization
- Rate limiting per user
- Security headers on responses
- SQL injection prevention through parameterized queries

## Testing

Run unit tests:
```bash
cd backend
python -m pytest tests/unit/
```

Run integration tests:
```bash
cd backend
python -m pytest tests/integration/
```

Run contract tests:
```bash
cd backend
python -m pytest tests/contract/
```

## Performance Considerations

- Responses are streamed to provide immediate feedback
- Conversation context is maintained across messages
- RAG retrieval is optimized for low latency
- Rate limiting prevents abuse

## Error Handling

The system implements comprehensive error handling:

- Graceful degradation when LLM or RAG services are unavailable
- Proper HTTP status codes for different error conditions
- Detailed logging for debugging purposes
- User-friendly error messages

## Deployment

For production deployment:

1. Use a production-grade database (PostgreSQL recommended)
2. Set up proper SSL certificates
3. Configure a reverse proxy (nginx/Apache)
4. Set appropriate environment variables for production
5. Monitor logs and performance metrics

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## License

[Specify your license here]