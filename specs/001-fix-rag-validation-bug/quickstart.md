# Quickstart: RAG Validation Bug Fix

## Overview
This guide explains how to implement the fix for the RAG validation bug where requests with mode="selected_text" and null/empty selected_text are incorrectly processed.

## Prerequisites
- Python 3.11+
- FastAPI
- Qdrant vector database
- Google Gemini API key
- Cohere API key (for embeddings)

## Files to Modify

### 1. Backend API Layer (`backend/api/chat.py`)
- Locate the `/ask` endpoint (around line 36)
- Update the validation logic for selected_text mode
- Ensure proper fallback handling and logging

### 2. RAG Model Layer (`backend/models/rag.py`)
- Locate the `process_query` method (around line 310)
- Add validation to prevent processing empty text in selected_text mode
- Ensure consistent behavior with API-level validation

## Implementation Steps

### Step 1: Update API Validation
```python
# In backend/api/chat.py
# Update the validation logic to properly handle empty selected_text
if request.mode == "selected_text":
    if not request.selected_text or not request.selected_text.strip():
        logger.info("Selected text mode requested but no text provided, switching to full_book mode")
        effective_mode = "full_book"
        sanitized_selected_text = None  # Ensure no empty text is passed
    else:
        # Process with selected_text mode
        sanitized_selected_text = sanitize_selected_text(request.selected_text)
```

### Step 2: Update RAG Model Validation
```python
# In backend/models/rag.py
# Add validation in process_query method
if mode == "selected_text":
    if not selected_text or not selected_text.strip():
        logger.warning("Selected text mode requested but no valid text provided, using full_book mode")
        # Fall back to full book mode
        context = await self.retrieve_context(query, top_k)
    else:
        # Use selected text mode
        context = await self.retrieve_context_from_text(query, selected_text)
```

### Step 3: Add Proper Logging
- Add INFO level logs when fallbacks occur
- Log the reason for the fallback
- Include relevant request parameters in logs

## Testing

### Test Cases
1. **Valid selected_text request**: mode="selected_text", selected_text="valid text"
2. **Empty selected_text request**: mode="selected_text", selected_text=""
3. **Null selected_text request**: mode="selected_text", selected_text=None
4. **Whitespace-only selected_text request**: mode="selected_text", selected_text="   "
5. **Valid full_book request**: mode="full_book" (should remain unchanged)

### Expected Behavior
- Requests with empty/null selected_text in selected_text mode should fallback to full_book mode
- Proper logging should occur when fallbacks happen
- Valid requests should continue to work as before
- Error responses should be appropriate if validation strategy changes to return 400 errors

## Deployment
1. Test the changes in a development environment
2. Run existing tests to ensure no regressions
3. Deploy to staging environment for further validation
4. Monitor logs for fallback behavior
5. Deploy to production after validation