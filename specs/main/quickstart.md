# Quickstart: Integrated RAG Chatbot for Physical AI and Humanoid Robotics Book

## Prerequisites

- Python 3.11+
- Node.js 18+ (for frontend development)
- Docker and Docker Compose
- Access to Gemini API (for embeddings and generation)
- Qdrant Cloud account (or local Qdrant instance)
- Neon Postgres account (or local Postgres instance)

## Setup Instructions

### 1. Clone and Navigate to Project

```bash
git clone <repository-url>
cd humanoid-robotics-book
```

### 2. Backend Setup

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Environment Configuration

Create a `.env` file in the backend directory with the following variables:

```env
# Qdrant Configuration
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_chunks

# Gemini Configuration
GEMINI_API_KEY=your_gemini_api_key
GEMINI_MODEL=gemini-2.5-flash

# Cohere (alternative embedding provider)
COHERE_API_KEY=your_cohere_api_key  # Optional

# Database Configuration
DATABASE_URL=your_neon_postgres_connection_string

# Application Configuration
BOOK_CONTENT_PATH=./doc
CHUNK_SIZE=512
CHUNK_OVERLAP=64
MAX_CONTEXT_LENGTH=2048

# Security
SECRET_KEY=your_secret_key
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
```

### 4. Database Setup

```bash
# Run the database migrations (if using Alembic)
alembic upgrade head

# Or manually execute the SQL schema
psql -d $DATABASE_URL -f ../configs/db.sql
```

### 5. Qdrant Setup

Ensure your Qdrant collection is created with the appropriate vector dimensions for your chosen embedding model.

### 6. Frontend Setup

```bash
# Navigate to frontend directory
cd frontend

# Install dependencies if needed (for development)
npm install  # if using build tools
```

### 7. Running the Application

#### Option 1: Using Docker Compose (Recommended)

```bash
# From the project root
docker-compose up --build
```

#### Option 2: Running Separately

Backend:
```bash
cd backend
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

Frontend:
```bash
# Serve the frontend files (using Python's built-in server as example)
cd frontend
python -m http.server 3000
```

## Initial Data Ingestion

1. Place your book content in the `doc/` directory
2. Call the `/ingest` endpoint to process the content:

```bash
curl -X POST http://localhost:8000/ingest
```

## API Endpoints

- `POST /ask` - Ask questions about the book content
- `POST /ingest` - Ingest book content (admin only)
- `GET /health` - Health check
- `GET /metadata` - Get book metadata
- `POST /auth/register` - User registration
- `POST /auth/login` - User login

## Testing

Run backend tests:
```bash
cd backend
pytest
```

## Development

For development, use the hot-reload mode:
```bash
cd backend
uvicorn main:app --reload
```

## Troubleshooting

1. **API Keys**: Ensure all required API keys are properly set in the environment
2. **Database Connection**: Verify the DATABASE_URL is correct and accessible
3. **Qdrant Connection**: Ensure Qdrant is accessible and the collection exists
4. **CORS**: If running frontend and backend on different ports, ensure CORS is configured