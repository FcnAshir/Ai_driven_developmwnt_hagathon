# Implementation Plan: ChatKit + Gemini + RAG Integration

**Branch**: `001-chatkit-gemini-rag-integration` | **Date**: 2025-12-10 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/001-chatkit-gemini-rag-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a ChatKit-based chatbot integrated with a FastAPI backend, Gemini LLM provider, and Retrieval-Augmented Generation (RAG) system for a digital book. The system provides contextual responses based on book content stored in Qdrant, with streaming responses, thread context maintenance, and custom actions.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, ChatKit SDK, Qdrant client, Google Generative AI SDK, Pydantic
**Storage**: Qdrant vector database for RAG, SQLite/PostgreSQL for thread storage
**Testing**: pytest for unit and integration tests
**Target Platform**: Linux server (web application backend)
**Project Type**: Web application (backend API with frontend integration)
**Performance Goals**: <5s response time for 95th percentile, streaming responses with initial feedback within 1 second
**Constraints**: <200ms RAG retrieval time, token limit management for LLM context windows, rate limiting per user
**Scale/Scope**: Support multiple concurrent users, handle token limits for long conversations

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the constitution document:
1. ✅ ChatKit widget integration with custom apiURL endpoint
2. ✅ FastAPI backend implementation with streaming responses
3. ✅ RAG pipeline integration with Qdrant vector database
4. ✅ Gemini LLM integration as non-OpenAI alternative
5. ✅ Thread persistence and metadata management
6. ✅ Security and privacy compliance with JWT authentication
7. ✅ Data residency control (no external API keys required for core functionality)

## Project Structure

### Documentation (this feature)

```text
specs/001-chatkit-gemini-rag-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
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
└── tests/
    ├── unit/
    ├── integration/
    └── contract/
```

**Structure Decision**: Web application structure selected with dedicated backend service for ChatKit integration, separating concerns between the chat interface, RAG pipeline, and LLM processing.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
