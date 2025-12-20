# API Documentation: Humanoid Robotics RAG System

## Overview

This API provides a Retrieval-Augmented Generation (RAG) system for humanoid robotics content, using Qdrant Cloud for vector storage and Google's Gemini model for response generation.

## Base URL

`http://localhost:8000` (or your deployed server URL)

## Endpoints

### Health Check

#### `GET /api/health`
Basic health check for the API service.

**Response:**
```json
{
  "status": "healthy",
  "timestamp": "2025-12-20T10:30:00Z"
}
```

#### `GET /api/health/qdrant`
Check connectivity to Qdrant Cloud.

**Response:**
```json
{
  "status": "healthy",
  "message": "Successfully connected to Qdrant Cloud",
  "timestamp": "2025-12-20T10:30:00Z",
  "qdrant_cloud_status": "connected"
}
```

### Diagnostic Information

#### `GET /api/diagnostic/qdrant`
Get diagnostic information about Qdrant Cloud collections.

**Response:**
```json
{
  "status": "success",
  "collections": [
    {
      "name": "humanoid_robotics_book",
      "vector_count": 1500,
      "indexed": true,
      "indexed_fields": ["content_vector"],
      "size": "245MB"
    }
  ],
  "qdrant_cloud_info": {
    "version": "1.6.0",
    "collections_count": 1,
    "total_vectors": 1500
  },
  "timestamp": "2025-12-20T10:30:00Z"
}
```

### Chat Functionality

#### `POST /api/ask`
Ask a question about humanoid robotics content.

**Request Body:**
```json
{
  "question": "What is humanoid robotics?",
  "mode": "full_book",
  "user_id": "optional_user_id"
}
```

**Response:**
```json
{
  "question": "What is humanoid robotics?",
  "answer": "Humanoid robotics is a branch of robotics focused on creating robots with human-like characteristics...",
  "sources": [
    {
      "content_id": "ch01_sec01",
      "text": "Definition of humanoid robotics from chapter 1",
      "similarity_score": 0.87
    }
  ],
  "mode": "full_book",
  "timestamp": "2025-12-20T10:30:00Z",
  "processing_time_ms": 2450
}
```

## Frontend Integration

### CORS Configuration

The API is configured to allow requests from common frontend development ports:
- `http://localhost:3000`
- `http://localhost:3001`
- `http://localhost:3002`
- `http://localhost:5173` (Vite default)
- And others

### API Client Example

See `api_client_example.py` for a complete implementation of how to interact with this API from a frontend application.

## Error Handling

- `200 OK`: Request successful
- `400 Bad Request`: Invalid request parameters
- `500 Internal Server Error`: Server-side error
- `503 Service Unavailable`: External service (Qdrant/Gemini) unavailable

## Response Time Goals

- Health endpoints: <5 seconds
- Chat endpoint: <10 seconds
- Diagnostic endpoint: <5 seconds