# API Contracts: API Endpoint Testing After Qdrant Cloud Migration

## Health Endpoint Contract

### GET /api/health/qdrant
**Purpose**: Confirm connectivity to Qdrant Cloud

#### Request
- **Method**: GET
- **Path**: /api/health/qdrant
- **Headers**: None required
- **Parameters**: None
- **Body**: None

#### Response
- **Success (200 OK)**:
  ```json
  {
    "status": "healthy",
    "message": "Successfully connected to Qdrant Cloud",
    "timestamp": "2025-12-20T10:30:00Z",
    "qdrant_cloud_status": "connected"
  }
  ```

- **Error (503 Service Unavailable)**:
  ```json
  {
    "status": "unhealthy",
    "message": "Failed to connect to Qdrant Cloud",
    "timestamp": "2025-12-20T10:30:00Z",
    "qdrant_cloud_status": "disconnected"
  }
  ```

## Diagnostic Endpoint Contract

### GET /api/diagnostic/qdrant
**Purpose**: Report collection status and vector information

#### Request
- **Method**: GET
- **Path**: /api/diagnostic/qdrant
- **Headers**: None required
- **Parameters**: None
- **Body**: None

#### Response
- **Success (200 OK)**:
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

- **Error (404 Not Found)**:
  ```json
  {
    "status": "error",
    "message": "Collections not found in Qdrant Cloud",
    "timestamp": "2025-12-20T10:30:00Z"
  }
  ```

## Chat Endpoint Contract

### POST /api/ask
**Purpose**: Process user queries against Qdrant Cloud using Gemini model

#### Request
- **Method**: POST
- **Path**: /api/ask
- **Headers**:
  - Content-Type: application/json
- **Parameters**: None
- **Body**:
  ```json
  {
    "question": "What is humanoid robotics?",
    "mode": "full_book",
    "user_id": "optional_user_id"
  }
  ```

#### Response
- **Success (200 OK)**:
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

- **Error (400 Bad Request)**:
  ```json
  {
    "status": "error",
    "message": "Invalid request parameters",
    "details": "Question field is required",
    "timestamp": "2025-12-20T10:30:00Z"
  }
  ```

- **Error (500 Internal Server Error)**:
  ```json
  {
    "status": "error",
    "message": "Failed to process request",
    "details": "Error connecting to Qdrant Cloud or Gemini API",
    "timestamp": "2025-12-20T10:30:00Z"
  }
  ```