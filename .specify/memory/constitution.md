<!-- Sync Impact Report:
Version change: 0.0.0 (initial) -> 1.0.0
Modified principles: All principles significantly modified or new ones introduced.
Added sections: All sections from the new constitution, including Executive summary, Goals & non-goals, High-level architecture, Components & responsibilities, Data contracts & schemas, ChatKit Server specification, RAG pipeline specification, Gemini integration (LLM) specification, Frontend ChatKit widget integration, Widgets, Actions, and Client Tools, Store & FileStore contract, Thread metadata, security & privacy, Dev workflow (Spec-Kit Plus driven), Acceptance criteria & tests, Deployment & infra, Monitoring, observability & SLOs, CI/CD, tests & release plan, Appendix: sample code snippets & API examples.
Removed sections: All sections from the previous constitution have been replaced.
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: None
-->
# Constitution: ChatKit + Gemini + RAG Integration

**Purpose:**
This document is the authoritative specification (the "Constitution") for implementing an advanced self-hosted ChatKit integration that uses a Retrieval-Augmented Generation (RAG) backend and a non-OpenAI LLM (Gemini) as the inference engine — built using a Spec-Driven workflow guided by Spec-Kit Plus principles.

---

## Table of contents

1. Executive summary
2. Goals & non-goals
3. High-level architecture
4. Components & responsibilities
5. Data contracts & schemas
6. ChatKit Server specification
7. RAG pipeline specification
8. Gemini integration (LLM) specification
9. Frontend ChatKit widget integration
10. Widgets, Actions, and Client Tools
11. Store & FileStore contract
12. Thread metadata, security & privacy
13. Dev workflow (Spec-Kit Plus driven)
14. Acceptance criteria & tests
15. Deployment & infra
16. Monitoring, observability & SLOs
17. CI/CD, tests & release plan
18. Appendix: sample code snippets & API examples

---

## 1. Executive summary

This document defines the interface and implementation plan so that the book's frontend shows an embedded ChatKit widget while the ChatKit server uses the project's RAG pipeline plus a Gemini LLM for final generation — no paid OpenAI API key required. The process is spec-driven (Spec-Kit Plus): a living spec guides each sprint, code is generated/validated against the spec, and reusable "intelligence" artifacts are captured.

<h2>2. Goals & non-goals</h2>

**Goals**

* Embed a ChatKit UI inside the published book.
* Route widget requests to a self-hosted ChatKit server.
* ChatKit server retrieves context from the RAG (Qdrant + Postgres) and calls Gemini for generation.
* Maintain complete control over data residency, auth, and observability.
* Use Spec-Kit Plus workflow: spec → plan → tasks → implement → validate.

**Non-goals**

* Using paid OpenAI models for inference.
* Replacing the existing RAG architecture: the RAG continues to be the primary retrieval system.

<h2>3. High-level architecture</h2>

```
[Frontend Book (ChatKit Widget)]
           ↓ (apiURL -> /chatkit)
[ChatKit Server (FastAPI)]
  ├─ Thread/store (SQLite/Postgres)  <-- metadata
  ├─ FileStore (S3 / signed URLs)
  ├─ RAG adapter (Qdrant client)
  └─ Gemini adapter (Gemini API / Gemini CLI / Ollama proxy)
           ↓
       [Gemini LLM]
```

Notes:

* ChatKit server exposes `/chatkit` for the client widget.
* The server calls `run_rag()` to fetch context from the vector DB.
* The LLM receives the RAG context + user query to produce final text.

<h2>4. Components & responsibilities</h2>

* **Frontend (book)**: Host ChatKit web component; provide `apiURL` and auth headers; place widget in UI; send page/chapter context if available.
* **ChatKit Server**: Map ChatKit protocol to internal handlers; implement `respond`, `action`, and streaming; call RAG and LLM adapters; persist threads/messages.
* **RAG Pipeline**: Responsible for retrieval, context assembly, source attribution, and shortlisting top-k passages.
* **LLM Adapter (Gemini)**: Responsible for safely prompting Gemini, streaming progress, and returning final answer with reasoning metadata (if supported).
* **Store/FileStore**: Persist messages, threads, and user files; support signed uploads and previews.
* **Spec-Kit Plus Controller/Artifacts**: Manage spec as living source-of-truth, store reusable prompts, evaluation rubrics, and test scenarios.

<h2>5. Data contracts & schemas</h2>

<h3>Thread Metadata (server-side only)</h3>

```json
{
  "thread_id": "uuid",
  "user_id": "string",
  "book_id": "string",           // optional
  "chapter_id": "string",        // optional
  "last_response_run_id": "string",
  "labels": ["string"]
}
```

<h3>Message Item (simplified)</h3>

```json
{
  "id": "uuid",
  "type": "user|assistant|tool|system",
  "content": [{"type":"text","text":"..."}],
  "created_at": "iso8601",
  "metadata": {}
}
```

<h3>RAG Response Contract (returned to ChatKit server)</h3>

```json
{
  "query": "...",
  "top_k": 5,
  "results": [
    {"id":"doc1","score":0.98,"source":"book:chapter1","text":"...","cursor":123},
    ...
  ],
  "assembled_context": "long string used for prompt",
  "retrieval_meta": {"method":"qdrant","params":{}}
}
```

<h2>6. ChatKit Server specification</h2>

* **Framework**: FastAPI (async) with StreamingResponse for SSE.

* **Endpoints**:

  * `POST /chatkit` — ChatKit protocol entrypoint; returns streaming events.
  * `GET /.well-known/health` — health check.
  * `POST /actions` — optional: action handling if client uses client-side handlers.

* **Server Class (`MyChatKitServer`)**:

  * Implement `respond(thread, input, context)` as async generator streaming `Event`s.
  * Implement `action(thread, action_payload, context)` to support widget actions.
  * Implement `to_message_content` to map uploaded file parts to internal storage.

* **Flow**:

  1. Receive user input event.
  2. Extract user text and optional page/chapter context.
  3. Call `run_rag(query,ctx)` → get assembled_context and source list.
  4. Build LLM prompt using a templated system prompt + assembled_context + user question.
  5. Call Gemini adapter with streaming; convert Gemini tokens to ChatKit streaming events.
  6. Persist assistant message in store.

* **Rate limiting & concurrency**: enforce per-user rate limit; limit concurrent LLM calls per server instance.

<h2>7. RAG pipeline specification</h2>

* **Inputs**: `query: str`, `context_filters: {book_id,chapter_id}`, `top_k: int`

* **Process**:

  1. Create embedding for `query` using same embeddings model used for book (or compatible model).
  2. Query Qdrant for top-k nearest vectors with metadata filters.
  3. Re-rank using a lightweight cross-encoder (optional).
  4. Assemble a context string limited to `N` tokens (configurable) with clear separators and source attributions.
  5. Return `RAG Response Contract` to ChatKit server.

* **Config params**:

  * `TOP_K` (default 8)
  * `MAX_CONTEXT_TOKENS` (default 2500)
  * `RE_RANKER_ENABLED` (bool)

<h2>8. Gemini integration (LLM) specification</h2>

* **Adapter** must support:

  * synchronous or streaming generation
  * stop sequences
  * token limits and chunked prompts
  * safe-content filters and redaction hooks
* **Prompt template**:

```
SYSTEM: You are an expert assistant for the book "{book_title}". Use the context below to answer concisely and cite sources.

CONTEXT:
{assembled_context}

USER QUESTION:
{user_question}

INSTRUCTIONS:
1. Answer clearly and concisely.
2. If uncertain, say so and provide sources from the context.
3. Provide short summary + 1–3 action items when applicable.
```

* **Gemini call**: include `max_tokens`, `temperature`, `top_p`, `stream` params from config.
* **Fallback path**: if Gemini fails, return a helpful system reply and log for retry.

<h2>9. Frontend ChatKit widget integration</h2>

* **Script**: use ChatKit web component from CDN.

* **Options**:

  * `apiURL` — `https://<your-domain>/chatkit`
  * `fetch` override — provide an auth interceptor to add JWT/Session header.
  * `initialThread` — open existing thread id when user opens a book/chapter.
  * `composer` — disable file uploads if not supported.

* **Context injection**: when mounting the widget, pass `book_id` and `chapter_id` via `initialThread` or as a hidden system message so server can filter RAG retrieval.

<h2>10. Widgets, Actions, and Client Tools</h2>

* **Widgets**: Card, List, Buttons, Forms used to present summaries, citations, and interactive follow-ups.
* **Actions**:

  * `cite-source` — open source location in the book viewer.
  * `save-note` — call client tool to save note to user's account.
* **Client Tools**: register any client-side action names in `clientTools` option.

<h2>11. Store & FileStore contract</h2>

* **Store**: use Postgres or SQLite for local dev. Must support:

  * CRUD threads
  * messages with content blobs
  * generate_item_id(item_type, thread, context)
* **FileStore**: implement signed URL flow for large uploads (S3/GCS). Provide preview endpoints.

<h2>12. Thread metadata, security & privacy</h2>

* **Auth**: JWT tokens from book frontend; validated at request time; `context` param to `server.process` must contain `user_id`.
* **PII**: Do not log raw user PII in plain text. Redact sensitive tokens/fields before logging.
* **Data retention**: default 90 days for ephemeral chat threads; configurable.
* **Encryption**: TLS in transit, optional at-rest encryption for stored contexts.

<h2>13. Dev workflow (Spec-Kit Plus driven)</h2>

* **Commands** (example) used with Spec-Kit Plus: `/constitution`, `/specify`, `/plan`, `/tasks`, `/implement`.
* **Artifacts**:

  * `CONSTITUTION.md` (this doc)
  * `IMPLEMENTATION_SPEC.json` (machine readable contract)
  * `TEST_SUITE.yaml` (scenarios and acceptance tests)
  * `PROMPT_LIBRARY/` (reusable prompt templates)
* **Sprints**: each sprint validates a portion of the spec — retrieval, LLM integration, streaming, widget behaviors.

<h2>14. Acceptance criteria & tests</h2>

* **End-to-end**: widget -> server -> RAG -> Gemini -> widget returns an answer with at least 1 source attribution.
* **Latency**: median LLM response < 3s (configurable); 95p < 10s.
* **Security**: JWT validated; logs scrubbed.
* **Quality**: For a 20-sample test-suite, at least 85% of answers must contain correct context citation and acceptable factuality per rubric.

<h2>15. Deployment & infra</h2>

* **Containers**: Docker images for server and optional Gemini proxy or Ollama adapter.
* **Scaling**: Horizontal for server, vector DB as managed Qdrant Cloud or sharded Qdrant.
* **Secrets**: store Gemini key in vault (e.g., Secrets Manager).

<h2>16. Monitoring, observability & SLOs</h2>

* **Metrics**:

  * requests/sec
  * LLM latency
  * RAG retrieval time
  * error rates
* **Logging**: structured logs, sample transcripts with redaction.
* **Alerts**: high error rate, high LLM latency, vector DB unavailable.

<h2>17. CI/CD, tests & release plan</h2>

* **Unit tests**: server handlers, RAG adapter, Gemini adapter.
* **Integration tests**: spin up local Qdrant, run sample flows.
* **E2E**: Cypress for frontend embedding + server SSE.
* **Release**: Canary → production; rollback if error rate > threshold.

<h2>18. Appendix: sample code snippets & API examples</h2>

* Minimal FastAPI chatkit endpoint (pseudocode) and example prompt templates are stored in `examples/` folder in the repo.

---

# Next steps

1. Review this constitution and mark sections to expand.
2. Run the Spec-Kit Plus `/specify` flow to transform sections into a machine-readable `IMPLEMENTATION_SPEC.json`.
3. Generate tasks and assign to sprints.

---

*Document autogenerated as a living spec. Update this file for any behavioral change.*

**Version**: 1.0.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-09
