# RAG System API

This is a Retrieval-Augmented Generation (RAG) system API that provides question-answering capabilities with comprehensive error handling and dependency validation.

## Features

- **Question Answering**: Process user questions against a knowledge base
- **Multiple Modes**: Support for both full-book search and selected-text processing
- **Comprehensive Error Handling**: Descriptive error messages instead of generic responses
- **Dependency Validation**: Health checks for vector database and embedding services
- **Null Parameter Handling**: Graceful handling of null/missing parameters
- **Structured Logging**: Detailed logging for debugging and monitoring

## Architecture

The system follows a service-oriented architecture with clear separation of concerns:

- **Models**: Pydantic models for request/response validation
- **Services**: Business logic for RAG processing, dependency checking, and embeddings
- **API**: FastAPI endpoints with centralized error handling
- **Middleware**: Custom error handlers and logging infrastructure

## Endpoints

### POST /api/ask
Submit a question to the RAG system for processing.

**Request Body:**
```json
{
  "question": "string (required)",
  "mode": "string (required, enum: 'full_book', 'selected_text')",
  "selected_text": "string (optional, can be null)"
}
```

**Success Response (200 OK):**
```json
{
  "response": "string",
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

**Error Response (4xx/5xx):**
```json
{
  "detail": "string",
  "error_code": "string (optional)",
  "timestamp": "string",
  "request_id": "string"
}
```

### GET /api/health
Check the health status of the RAG system and its dependencies.

**Response (200 OK):**
```json
{
  "status": "healthy|degraded",
  "timestamp": "string",
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

## Setup

1. **Install Dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Configure Environment:**
   Create a `.env` file based on `.env.example`:
   ```bash
   cp .env.example .env
   # Edit .env with your configuration
   ```

3. **Start the Server:**
   ```bash
   python -m src.api.main
   # or
   uvicorn src.api.main:app --reload --port 8000
   ```

## Environment Variables

- `QDRANT_URL`: URL for the Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant (if required)
- `QDRANT_COLLECTION_NAME`: Name of the collection to query
- `COHERE_API_KEY`: API key for Cohere embedding service
- `GEMINI_API_KEY`: API key for Google Generative AI
- `API_HOST`: Host for the API server (default: 0.0.0.0)
- `API_PORT`: Port for the API server (default: 8000)
- `API_DEBUG`: Enable debug mode (default: False)

## Error Handling

The system provides descriptive error messages for various failure scenarios:

- **400 Bad Request**: Invalid request parameters
- **422 Unprocessable Entity**: Request validation failures
- **500 Internal Server Error**: Unexpected server errors
- **502 Bad Gateway**: External service unavailability
- **503 Service Unavailable**: Dependency unavailability

## Testing

Run the tests using pytest:

```bash
# Run all tests
pytest

# Run specific test files
pytest tests/unit/
pytest tests/integration/
pytest tests/contract/
```

## Security Considerations

- Error messages do not expose internal system details
- API keys are handled securely via environment variables
- Input validation prevents injection attacks
- Request correlation IDs for debugging without exposing sensitive data