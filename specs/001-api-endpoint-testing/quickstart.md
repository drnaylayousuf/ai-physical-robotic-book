# Quickstart Guide: API Endpoint Testing After Qdrant Cloud Migration

## Prerequisites

- Python 3.11+
- Qdrant Cloud account and credentials
- Google Gemini API key
- Git

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd humanoid-robotics-book
   ```

2. **Set up Python environment**
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

3. **Configure environment variables**
   ```bash
   cp backend/.env.example backend/.env
   # Edit backend/.env with your Qdrant Cloud and Gemini API credentials
   ```

4. **Install backend dependencies**
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

## Running Tests

### 1. Start the API Server
```bash
cd backend
python main.py
# Server will run on http://localhost:8000
```

### 2. Test Health Endpoint
```bash
curl -X GET http://localhost:8000/api/health/qdrant
```

Expected response:
```json
{
  "status": "healthy",
  "message": "Successfully connected to Qdrant Cloud",
  "timestamp": "2025-12-20T10:30:00Z",
  "qdrant_cloud_status": "connected"
}
```

### 3. Test Diagnostic Endpoint
```bash
curl -X GET http://localhost:8000/api/diagnostic/qdrant
```

Expected response:
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

### 4. Test Chat Endpoint
```bash
curl -X POST http://localhost:8000/api/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is humanoid robotics?",
    "mode": "full_book"
  }'
```

Expected response:
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

## Verification Steps

1. **Verify Qdrant Cloud Connectivity**: Check that the health endpoint returns "connected" status
2. **Verify Collection Status**: Check that the diagnostic endpoint shows proper collection information
3. **Verify Chat Functionality**: Test that the chat endpoint returns relevant answers using book content
4. **Verify Gemini Integration**: Confirm that responses are generated using the Gemini model (not OpenAI)
5. **Verify Migration Success**: Ensure that all queries are served from Qdrant Cloud instead of in-memory storage

## Troubleshooting

- **Qdrant Cloud Connection Issues**: Verify your QDRANT_API_KEY and cluster URL in the .env file
- **Gemini API Issues**: Verify your GEMINI_API_KEY in the .env file
- **Endpoint Not Found**: Ensure the server is running on http://localhost:8000
- **Slow Response Times**: Check your network connection to Qdrant Cloud and Gemini API