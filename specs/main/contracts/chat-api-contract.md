# API Contract: Chat Endpoint

## Overview
This contract defines the `/api/ask` endpoint that the frontend UI calls to get responses from the RAG system.

## Endpoint
`POST /api/ask`

## Request
### Headers
- `Content-Type: application/json`

### Body
```json
{
  "question": "string",
  "mode": "full_book|selected_text",
  "selected_text": "string (optional)",
  "user_id": "string (optional)"
}
```

### Parameters
- `question` (required): The question to ask about the book content
- `mode` (required): The mode to use for retrieval
  - `full_book`: Search the entire book content
  - `selected_text`: Only use the provided selected_text for context
- `selected_text` (optional): Text selected by the user (required when mode is `selected_text`)
- `user_id` (optional): Identifier for the user making the request

## Response
### Success (200 OK)
```json
{
  "response": "string",
  "sources": [
    {
      "chunk_id": "string",
      "content": "string",
      "score": "float",
      "source": "string"
    }
  ],
  "references": ["string"]
}
```

### Error Responses
- `400 Bad Request`: Invalid request parameters
- `500 Internal Server Error`: Server error during processing

## Expected Behavior
When properly configured with ingested content and generative model:
1. The endpoint should return a relevant response to the user's question
2. Sources should contain relevant chunks from the book content
3. References should contain relevant section identifiers

## Current Issue Behavior
When not properly configured (no content or model):
- Returns: "The book does not provide details about this topic. No context is available and no generative model is configured."
- Sources and references are empty

## Configuration Requirements
- Qdrant vector database must contain ingested book content
- GEMINI_API_KEY must be properly configured for generative model
- Backend must be running on the correct port (8000 for UI)