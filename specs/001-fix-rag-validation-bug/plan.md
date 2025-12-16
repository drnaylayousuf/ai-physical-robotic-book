# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Fix the RAG validation bug where requests with mode="selected_text" and null/empty selected_text are incorrectly processed, causing Gemini to receive insufficient context. The solution involves implementing strict validation to either automatically fallback to mode="full_book" or return a 400 validation error when selected_text is null or empty in selected_text mode. The fix will be implemented in the backend RAG API endpoint and potentially in the RAG model processing logic.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, Qdrant, Cohere, Google Gemini 2.5 Flash, Pydantic
**Storage**: Qdrant vector database (for RAG retrieval)
**Testing**: pytest
**Target Platform**: Linux server (web backend)
**Project Type**: Web backend service
**Performance Goals**: Maintain existing RAG response times under 2s p95, no degradation in valid requests
**Constraints**: Must maintain backward compatibility for valid requests, proper error handling for invalid requests
**Scale/Scope**: Single backend service handling RAG requests

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Decision Point Mapping
- **Validation Strategy**: Automatic fallback to "full_book" mode when selected_text is empty/null (resolved: maintain existing approach with bug fix)
- **Logging Strategy**: INFO level logging when fallbacks occur from selected_text to full_book mode (resolved: clear logging implemented)

### Reasoning Activation
- **Input Validation**: Proper validation of RAG request parameters before processing (implemented)
- **Error Handling**: Appropriate error responses for invalid inputs (maintained existing behavior)

### Intelligence Accumulation
- **Reusable Validation Logic**: Leverage existing validation functions in sanitization utilities (reused existing code)
- **Logging Framework**: Use consistent logging approach following existing patterns (consistent with project)

### Right Altitude
- **Just Right**: Implementation focuses on the specific validation bug without over-engineering (maintained appropriate scope)

### Frameworks Over Rules
- **Conditional Reasoning**: "If mode is 'selected_text' and selected_text is empty/null, then fallback to full_book mode" (implemented)

### Meta-Awareness Against Convergence
- **Avoid Generic Solutions**: Focused specifically on the RAG validation bug (maintained specific focus)

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
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
├── main.py                    # FastAPI application entry point
├── api/
│   ├── chat.py               # RAG chat endpoints (contains /api/ask)
│   ├── health.py             # Health check endpoints
│   ├── ingestion.py          # Content ingestion endpoints
│   ├── metadata.py           # Metadata endpoints
│   ├── auth.py               # Authentication endpoints
│   └── admin.py              # Admin endpoints
├── models/
│   ├── rag.py                # RAG model implementation
│   └── database.py           # Database models and connections
├── utils/
│   ├── embeddings.py         # Embedding services
│   ├── sanitization.py       # Input sanitization utilities
│   ├── cache.py              # Caching utilities
│   └── monitoring.py         # Monitoring utilities
├── config/
│   └── settings.py           # Configuration settings
└── services/                 # Additional services
```

**Structure Decision**: Web backend service with FastAPI framework. The primary file to modify is backend/api/chat.py for the validation logic and potentially backend/models/rag.py for the RAG model processing logic.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
