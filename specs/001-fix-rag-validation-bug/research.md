# Research: RAG Validation Bug Fix

## Decision: Validation Strategy
**Rationale**: The current implementation in `backend/api/chat.py` already has some validation logic that falls back to "full_book" mode when selected_text is empty, but there's a bug in the RAG model that still processes empty text in selected_text mode. The fix should implement strict validation at both the API level and the RAG model level to ensure consistent behavior.

**Alternatives considered**:
1. Return 400 error when selected_text is empty in selected_text mode
2. Automatically fallback to full_book mode (current approach with bug fix)
3. Implement both strategies with configurable behavior

**Chosen approach**: Fix the existing fallback mechanism to work properly, as this provides the best user experience without breaking existing clients.

## Decision: Logging Strategy
**Rationale**: The system should log when fallbacks occur so operators can monitor usage patterns and potentially identify when users are submitting invalid requests. Clear logging will help with debugging and monitoring.

**Chosen approach**: Add INFO level logging when fallbacks occur from selected_text to full_book mode, with clear message indicating the reason for fallback.

## Decision: Python Version
**Rationale**: Based on the pyproject.toml file, the project uses Python 3.11.

**Chosen approach**: Python 3.11 is confirmed as the target version.

## Decision: Testing Framework
**Rationale**: Looking at the project structure, there are test files like `test_api_ask.py`, `test_rag_cohere_qdrant.py`, etc., and a `tests/` directory, indicating pytest is used.

**Chosen approach**: pytest is the testing framework.

## Technical Findings

### Current Bug Location
The bug exists in two places:
1. **API Layer** (`backend/api/chat.py:68-82`): The fallback logic exists but has an issue where it still passes potentially empty text to the RAG model
2. **RAG Model Layer** (`backend/models/rag.py:320-329`): The `process_query` method doesn't properly validate selected_text before calling `retrieve_context_from_text`

### Root Cause
When mode is "selected_text" but selected_text is empty/null, the API layer correctly sets effective_mode to "full_book", but it still passes the empty selected_text to the RAG model. The RAG model then calls `retrieve_context_from_text` with empty text, which can cause the Gemini model to receive insufficient context.

### Solution Approach
1. Fix the API layer to ensure empty selected_text is not passed to the RAG model when falling back to full_book mode
2. Add additional validation in the RAG model to prevent processing empty text in selected_text mode
3. Ensure proper logging when fallbacks occur