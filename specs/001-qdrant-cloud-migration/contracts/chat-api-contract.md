# Chat API Contract: Qdrant Cloud Integration

## Overview
The chat API endpoint that integrates with Qdrant Cloud for book content retrieval and response generation.

## Endpoints

### POST /chat
Process user query against book content stored in Qdrant Cloud

#### Request
```json
{
  "message": "string (user's question about book content)",
  "session_id": "string (optional, for conversation context)",
  "similarity_threshold": "float (optional, default: 0.7)"
}
```

#### Response
```json
{
  "response": "string (AI-generated response based on book content)",
  "sources": [
    {
      "id": "string (chunk ID)",
      "content": "string (relevant book content)",
      "page": "integer (page reference)",
      "section": "string (section reference)",
      "similarity_score": "float (0.0-1.0 similarity to query)"
    }
  ],
  "query_id": "string (unique identifier for this query)",
  "timestamp": "datetime (when the query was processed)"
}
```

#### Error Responses
- `400 Bad Request`: Invalid request format
- `503 Service Unavailable`: Qdrant Cloud unavailable and no fallback

#### Headers
- `Content-Type: application/json`
- `Authorization: Bearer <token>` (if authentication required)

## Dependencies
- Qdrant Cloud service
- Cohere API for embeddings
- Book content stored in Qdrant collection

## Performance Requirements
- Response time under 3 seconds for typical queries
- 99% availability of retrieval functionality
- Support for concurrent users

## Security Considerations
- API key validation for Qdrant Cloud
- Rate limiting to prevent abuse
- Input sanitization for user queries