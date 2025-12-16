# API Contract: Chat Endpoint

## Endpoint
`POST /api/ask`

## Description
Handles RAG-based question answering with two modes: full-book RAG mode and selected-text RAG mode based on user-selected text. Enhanced to provide consistent responses regardless of mode used.

## Request

### Headers
- `Content-Type: application/json`

### Body
```json
{
  "question": {
    "type": "string",
    "required": true,
    "minLength": 3,
    "maxLength": 1000,
    "description": "The question to be answered by the RAG system"
  },
  "mode": {
    "type": "string",
    "required": true,
    "enum": ["full_book", "selected_text"],
    "description": "The mode to use for context retrieval"
  },
  "selected_text": {
    "type": "string",
    "required": false,
    "nullable": true,
    "description": "Optional text selected by the user for context in selected_text mode"
  },
  "user_id": {
    "type": "string",
    "required": false,
    "nullable": true,
    "description": "Optional user identifier for tracking purposes"
  }
}
```

### Example Request
```json
{
  "question": "What is NVIDIA Isaac Platform?",
  "mode": "selected_text",
  "selected_text": "NVIDIA Isaac Platform is a robotics development solution...",
  "user_id": "user123"
}
```

## Response

### Success Response (200 OK)
```json
{
  "response": {
    "type": "string",
    "description": "The generated response from the AI model"
  },
  "sources": {
    "type": "array",
    "items": {
      "type": "object",
      "properties": {
        "chunk_id": {"type": "string"},
        "content": {"type": "string"},
        "score": {"type": "number"},
        "source": {"type": "string"}
      }
    },
    "description": "List of source chunks used to generate the response"
  },
  "references": {
    "type": "array",
    "items": {"type": "string"},
    "description": "List of unique source references for attribution"
  }
}
```

### Example Response
```json
{
  "response": "The NVIDIA Isaac Platform is a comprehensive, end-to-end robotics development solution...",
  "sources": [
    {
      "chunk_id": "selected_text_chunk_0",
      "content": "NVIDIA Isaac Platform is a robotics development solution...",
      "score": 0.85,
      "source": "selected_text_input"
    },
    {
      "chunk_id": "42",
      "content": "The NVIDIA Isaac Platform provides tools for robotics development...",
      "score": 0.72,
      "source": "robotics_handbook_ch3"
    }
  ],
  "references": [
    "selected_text_input",
    "robotics_handbook_ch3"
  ]
}
```

### Error Responses

#### 400 Bad Request
- Invalid question format (less than 3 characters or more than 1000 characters)
- Invalid mode (not "full_book" or "selected_text")

#### 503 Service Unavailable
- Vector database connection issues

#### 429 Too Many Requests
- Gemini API quota exceeded

## Changes from Original
1. Enhanced response consistency across both modes
2. Improved source attribution with relevance scores
3. Better error handling and fallback responses
4. Maintained backward compatibility with existing contracts