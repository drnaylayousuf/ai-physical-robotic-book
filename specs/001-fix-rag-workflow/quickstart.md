# Quickstart Guide: RAG System Error Handling

## Overview
This guide will help you set up and run the RAG system with enhanced error handling capabilities. The system provides question-answering functionality with comprehensive error reporting and dependency validation.

## Prerequisites
- Python 3.11 or higher
- Docker and Docker Compose (for running Qdrant)
- Access to Cohere API (for embeddings)
- Access to Google Generative AI API (for LLM responses)

## Setup

### 1. Environment Configuration
Create a `.env` file in the project root with the following variables:

```bash
COHERE_API_KEY=your_cohere_api_key_here
GEMINI_API_KEY=your_gemini_api_key_here
QDRANT_URL=http://localhost:6333
```

### 2. Install Dependencies
```bash
pip install fastapi uvicorn qdrant-client cohere google-generativeai pydantic python-dotenv
```

### 3. Start Vector Database
```bash
# Option 1: Using Docker
docker run -d --name qdrant -p 6333:6333 qdrant/qdrant

# Option 2: Using Docker Compose
docker-compose up -d qdrant
```

## Running the API

### 1. Start the Server
```bash
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

### 2. Test the API
```bash
# Test a basic question
curl -X POST "http://localhost:8000/api/ask" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are humanoid robots used for?",
    "mode": "full_book",
    "selected_text": null
  }'
```

### 3. Check Health Status
```bash
curl -X GET "http://localhost:8000/api/health"
```

## Error Handling Examples

### 1. Testing Vector Database Unavailability
If Qdrant is not running, the API will return:
```json
{
  "detail": "Vector database is not available. Please check if Qdrant is running and accessible.",
  "error_code": "VECTOR_DB_UNAVAILABLE",
  "timestamp": "2025-12-11T14:30:00Z",
  "request_id": "req_12345"
}
```

### 2. Testing Invalid Request Parameters
For a request with empty question:
```json
{
  "detail": "Question cannot be empty or contain only whitespace",
  "error_code": "INVALID_REQUEST",
  "timestamp": "2025-12-11T14:30:00Z",
  "request_id": "req_12346"
}
```

### 3. Testing Null Parameter Handling
The system properly handles null `selected_text` values without crashing:
```bash
curl -X POST "http://localhost:8000/api/ask" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the benefits of humanoid robots?",
    "mode": "full_book",
    "selected_text": null
  }'
```

## Key Features

### 1. Comprehensive Error Messages
Instead of generic "Error processing your question", you'll receive specific error details:
- Dependency unavailability
- Invalid request parameters
- Service timeouts
- Authentication failures

### 2. Dependency Validation
The system checks:
- Vector database connectivity
- Embedding service availability
- Collection existence

### 3. Null Parameter Handling
Safe handling of null values, particularly `selected_text`, preventing system crashes.

### 4. Health Check Endpoint
Monitor system status and dependencies with the `/health` endpoint.

## Troubleshooting

### Common Issues

1. **Qdrant Connection Error**: Ensure Qdrant is running on the configured URL
2. **API Key Issues**: Verify Cohere and Gemini API keys are valid
3. **Collection Not Found**: Make sure the expected Qdrant collection exists
4. **Embedding Generation Failure**: Check Cohere API access and rate limits

### Debugging Tips
- Check the logs for detailed error information
- Use the health endpoint to verify dependency status
- Verify environment variables are correctly set
- Test individual components separately