# Integrated RAG Chatbot for Physical AI and Humanoid Robotics Book

## Project Overview
Develop and embed a Retrieval-Augmented Generation (RAG) chatbot into a published book on physical AI and humanoid robotics. The chatbot will use the book's content to answer user questions, including queries based on user-selected text. All development must be done exclusively using Speckit Plus tools and workflows.

## Clarifications

### Session 2025-12-09
- Q: What authentication and authorization mechanism should be implemented for the API? → A: Full authentication with user accounts and role-based access control
- Q: What error handling and fallback strategy should be implemented for external service failures? → A: Comprehensive error handling with multiple fallback strategies and graceful degradation
- Q: What user roles and permissions should be implemented in the system? → A: Multiple roles: Admin, Moderator, Registered User, Guest with different permissions
- Q: What monitoring, logging, and observability approach should be implemented? → A: Comprehensive monitoring with metrics, logs, and tracing across all components
- Q: What performance targets and scalability approach should be implemented? → A: Specific performance targets: <2s response time, 1000+ concurrent users, horizontal scaling

## Requirements Analysis

### 1. Framework and SDKs
- **Primary Language Model**: Gemini 2.5 Flash
- **Integration**: Use OpenAI Agents/ChatKit SDKs but route requests to Gemini 2.5 Flash
- **Backend Framework**: FastAPI
- **Frontend Integration**: ChatKit-JS through context7 MCP
- **Backend Integration**: ChatKit-Python through context7 MCP

### 2. Backend Architecture
- **Framework**: FastAPI
- **Authentication**: Full authentication with user accounts and role-based access control
- **API Endpoints**:
  - `/ingest` - Upload and vectorize book content from `doc/` folder (admin access only)
  - `/ask` - Handle RAG-based question answering with two modes:
    - Full-book RAG mode (registered user and above)
    - Restricted RAG mode (registered user and above)
  - `/health` - Health check endpoint (public access)
  - `/metadata` - Book metadata and chapter information (public access)
  - `/auth/register` - User registration endpoint (guest and above)
  - `/auth/login` - User authentication endpoint (guest and above)
  - `/auth/profile` - User profile management (registered user and above)
  - `/admin` - Administrative functions (admin access only)
  - `/moderate` - Content moderation (moderator and admin access)
- **RAG Pipeline**: clean → chunk → embed → store → retrieve → generate
- **Environment Configuration**: Store Qdrant API key and other secrets in `.env` file

### 3. Database & Storage
- **Vector Database**: Qdrant Cloud Free Tier for vector storage and retrieval
- **Metadata Storage**: Neon Serverless Postgres for:
  - User queries history
  - User-selected text storage
  - Chapter metadata and book structure
- **Embeddings**: Use either Cohere embeddings or Gemini embeddings (free tier compatible)

### 4. Frontend Features
- **UI Components**:
  - Interactive chat interface
  - Text selection highlighting tool
  - Citation display with chunk IDs and source paragraphs
  - Streaming response capability
  - Book navigation integration
- **Integration**: Embedded via iframe/widget or direct integration with book content
- **Responsive Design**: Mobile and desktop compatibility

### 5. RAG Logic & Constraints
- **Hallucination Prevention**: Strictly use only retrieved content for answers
- **Source Attribution**: Reference paragraph/section IDs in responses
- **Restricted Mode**: When using user-selected text, only answer from that specific content
- **Quality Control**: Implement confidence scoring and fallback mechanisms
- **Error Handling**: Comprehensive error handling with multiple fallback strategies and graceful degradation for external service failures (Qdrant, Gemini, Cohere, Neon Postgres)

### 6. Deployment & Infrastructure
- **Backend**: Deployable to Railway or Render
- **Frontend**: Deployable to Vercel
- **Containerization**: Dockerfile and docker-compose for local development
- **Environment Templates**: `.env.example` with all required variables

### 7. Observability & Monitoring
- **Logging**: Comprehensive logging of all system events, errors, and user interactions
- **Metrics**: Collection of key performance indicators (response times, error rates, throughput)
- **Tracing**: Distributed tracing across all components for debugging and performance analysis
- **Alerting**: Alerting system for critical errors and performance degradation
- **Dashboard**: Real-time monitoring dashboard for system health and performance

## Technical Architecture

### System Components
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Book Content  │────│   Backend API    │────│   Frontend UI   │
│   (doc folder)  │    │   (FastAPI)      │    │   (ChatKit-JS)  │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                            │    │
                    ┌───────┘    └───────┐
                    ▼                   ▼
         ┌─────────────────┐    ┌──────────────────┐
         │   Qdrant        │    │  Neon Postgres   │
         │   Vector DB     │    │   Metadata DB    │
         └─────────────────┘    └──────────────────┘
```

### Data Flow
1. **Ingestion Flow**: Book files → Text extraction → Cleaning → Chunking → Embedding → Vector storage in Qdrant
2. **Query Flow**: User question → Vectorization → Similarity search → Context retrieval → Answer generation → Response streaming

## Detailed Specifications

### API Endpoints
#### POST `/ingest`
- Accepts book content from `doc/` folder
- Processes and vectorizes content
- Stores in Qdrant collection
- Returns ingestion status

#### POST `/ask`
- **Request Body**:
  ```json
  {
    "question": "string",
    "mode": "full_book|selected_text",
    "selected_text": "optional string for restricted mode",
    "user_id": "optional string"
  }
  ```
- **Response**:
  ```json
  {
    "response": "generated answer",
    "sources": [{"chunk_id": "string", "content": "string", "confidence": "float"}],
    "references": ["paragraph_ids"]
  }
  ```

#### GET `/health`
- Returns system status

#### GET `/metadata`
- Returns book structure and chapter information

### Database Schemas

#### Neon Postgres Tables
```sql
-- User authentication and profiles
CREATE TABLE users (
    id SERIAL PRIMARY KEY,
    username VARCHAR(255) UNIQUE NOT NULL,
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    role VARCHAR(50) DEFAULT 'user', -- Possible values: 'admin', 'moderator', 'user', 'guest'
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- User queries history
CREATE TABLE user_queries (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id),
    question TEXT,
    response TEXT,
    sources JSONB,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- User selected text
CREATE TABLE user_selected_text (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id),
    selected_text TEXT,
    context_metadata JSONB,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Chapter metadata
CREATE TABLE chapter_metadata (
    id SERIAL PRIMARY KEY,
    chapter_title VARCHAR(255),
    file_path VARCHAR(500),
    content_summary TEXT,
    paragraph_count INTEGER,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

#### Qdrant Collections
- `book_chunks`: Contains book content chunks with embeddings
- `selected_text_chunks`: Optional collection for user-selected text (if needed)

### Environment Variables
```
# Qdrant Configuration
QDRANT_URL=
QDRANT_API_KEY=
QDRANT_COLLECTION_NAME=book_chunks

# Gemini Configuration
GEMINI_API_KEY=
GEMINI_MODEL=gemini-2.5-flash

# Cohere (alternative embedding provider)
COHERE_API_KEY=

# Database Configuration
DATABASE_URL= # Neon Postgres connection string

# Application Configuration
BOOK_CONTENT_PATH=./doc
CHUNK_SIZE=512
CHUNK_OVERLAP=64
MAX_CONTEXT_LENGTH=2048
```

## Implementation Phases

### Phase 1: Infrastructure Setup
- Set up backend folder structure
- Configure Qdrant Cloud and Neon Postgres
- Create API endpoints skeleton
- Set up environment configuration

### Phase 2: RAG Pipeline Development
- Implement document parsing from `doc/` folder
- Build chunking and embedding pipeline
- Integrate vector storage with Qdrant
- Develop retrieval and generation logic

### Phase 3: Frontend Integration
- Implement ChatKit-JS interface
- Add text selection and highlighting
- Create citation display components
- Implement streaming responses

### Phase 4: Advanced Features
- Implement restricted RAG mode
- Add quality control mechanisms
- Implement confidence scoring
- Add comprehensive error handling

### Phase 5: Testing and Deployment
- Create comprehensive test suite
- Implement CI/CD pipelines
- Deploy to production environments
- Create documentation and user guides

## Quality Assurance

### Testing Requirements
- Unit tests for RAG pipeline components
- Integration tests for API endpoints
- End-to-end tests for chat functionality
- Performance tests for response times
- Accuracy tests for information retrieval

### Success Metrics
- **Accuracy**: >95% of responses based on book content
- **Latency**: <2 seconds average response time (target: <1 second for 95th percentile)
- **Throughput**: Support 1000+ concurrent users
- **Scalability**: Horizontal scaling capability
- **Reliability**: >99% uptime
- **User Satisfaction**: >4.0/5.0 rating

### Constraints & Limitations
- Must operate within free tier limits of Qdrant, Cohere, and Neon
- No hallucinations - responses must be grounded in book content
- Respectful and professional responses only
- Privacy compliance for user data

## Deliverables

### Backend Artifacts
- `backend/main.py` - FastAPI application entry point
- `backend/api/chat.py` - Chat endpoints
- `backend/api/ingestion.py` - Document ingestion endpoints
- `backend/models/rag.py` - RAG pipeline implementation
- `backend/models/database.py` - Database models and connections
- `backend/utils/embeddings.py` - Embedding utilities
- `backend/config/settings.py` - Configuration management
- `backend/Dockerfile` - Container configuration
- `docker-compose.yml` - Local development setup

### Frontend Artifacts
- `frontend/index.html` - Main chat interface
- `frontend/style.css` - Styling for chat interface
- `frontend/script.js` - ChatKit-JS integration
- `frontend/components/` - Reusable UI components

### Documentation
- `README.md` - Comprehensive setup and usage guide
- `ARCHITECTURE.md` - System architecture documentation
- `DEPLOYMENT.md` - Production deployment guide
- `TESTING.md` - Testing procedures and requirements

### Testing Assets
- `tests/postman_collection.json` - Postman collection
- `tests/curl_examples.sh` - cURL examples
- `tests/frontend_qa_checklist.md` - Frontend QA checklist

## Risk Assessment

### Technical Risks
- **Model Availability**: Gemini 2.5 Flash availability and rate limits
- **Vector Database Limits**: Qdrant Cloud free tier limitations
- **Performance**: Large book content affecting response times
- **Embedding Quality**: Cohere/Gemini embedding effectiveness

### Mitigation Strategies
- Implement caching for frequently asked questions
- Optimize chunk sizes and retrieval algorithms
- Use fallback mechanisms for API failures
- Monitor and optimize costs within free tiers

## Integration Points

### Book Integration
- Embed chatbot as iframe in book pages
- Create widget that overlays on book content
- Direct integration with book navigation

### External Services
- Qdrant Cloud for vector storage
- Neon Postgres for metadata
- Gemini API for inference
- Optional: Cohere for embeddings

This specification provides a comprehensive foundation for implementing the RAG chatbot while maintaining focus on the physical AI and humanoid robotics book content.