# API Contract: RAG System Error Handling

## Overview
This document defines the API contract for the RAG (Retrieval-Augmented Generation) system with enhanced error handling capabilities. The API provides a question-answering service that retrieves relevant information from a vector database and generates responses using an LLM.

## Base URL
`http://localhost:8000/api`

## Endpoints

### POST /ask
Submit a question to the RAG system for processing.

#### Request
**Content-Type**: `application/json`

**Body**:
```json
{
  "question": "string (required)",
  "mode": "string (required, enum: 'full_book', 'selected_text')",
  "selected_text": "string (optional, can be null)"
}
```

**Example**:
```json
{
  "question": "What are humanoid robots used for?",
  "mode": "full_book",
  "selected_text": null
}
```

#### Response
**Success Response (200 OK)**:
```json
{
  "response": "string (generated answer)",
  "sources": [
    {
      "chunk_id": "string",
      "content": "string",
      "score": "number",
      "source": "string"
    }
  ],
  "references": ["string"]
}
```

**Error Response (4xx/5xx)**:
```json
{
  "detail": "string (descriptive error message)",
  "error_code": "string (optional, machine-readable code)",
  "timestamp": "string (ISO 8601 datetime)",
  "request_id": "string (optional, correlation ID)"
}
```

#### Error Scenarios
- **400 Bad Request**: Invalid request parameters (empty question, invalid mode)
- **503 Service Unavailable**: Vector database or embedding service unavailable
- **500 Internal Server Error**: Unexpected error during processing

### GET /health
Check the health status of the RAG system and its dependencies.

#### Response
**Success Response (200 OK)**:
```json
{
  "status": "healthy",
  "timestamp": "string (ISO 8601 datetime)",
  "dependencies": {
    "vector_database": {
      "status": "available|unavailable",
      "details": "string (optional)"
    },
    "embedding_service": {
      "status": "available|unavailable",
      "details": "string (optional)"
    }
  }
}
```

**Error Response (503 Service Unavailable)**:
```json
{
  "detail": "string (descriptive error message)",
  "timestamp": "string (ISO 8601 datetime)"
}
```

## Error Handling Standards

### HTTP Status Codes
- **200 OK**: Request processed successfully
- **400 Bad Request**: Client sent invalid request parameters
- **422 Unprocessable Entity**: Request parameters failed validation
- **500 Internal Server Error**: Unexpected server error
- **502 Bad Gateway**: External service (vector database, embedding service) unavailable
- **503 Service Unavailable**: Service temporarily unavailable

### Error Response Format
All error responses follow the standard format:
```json
{
  "detail": "Human-readable error message",
  "error_code": "Optional machine-readable error code",
  "timestamp": "ISO 8601 datetime string",
  "request_id": "Optional correlation ID"
}
```

## Security Considerations
- Error messages should not expose internal system details
- API keys and sensitive information must not be included in error responses
- Input validation to prevent injection attacks