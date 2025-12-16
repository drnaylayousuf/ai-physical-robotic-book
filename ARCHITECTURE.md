# System Architecture Documentation

## Overview

The Integrated RAG Chatbot for Physical AI and Humanoid Robotics Book is a web application that combines a Docusaurus-based book with a RAG (Retrieval-Augmented Generation) chatbot system. The architecture follows a microservices approach with a FastAPI backend, vector database for content storage, and frontend for user interaction.

## High-Level Architecture

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

## Component Architecture

### Backend Services

The backend is built with FastAPI and provides the following services:

1. **Authentication Service** (`backend/api/auth.py`): Handles user registration, login, and JWT token management
2. **Chat Service** (`backend/api/chat.py`): Manages the RAG chat functionality
3. **Ingestion Service** (`backend/api/ingestion.py`): Handles book content ingestion and vectorization
4. **Metadata Service** (`backend/api/metadata.py`): Provides book structure and chapter information

### Data Layer

1. **Vector Database (Qdrant)**: Stores book content as vector embeddings for semantic search
   - Collection: `book_chunks`
   - Contains: Content chunks with embeddings, metadata

2. **Relational Database (PostgreSQL)**: Stores application metadata and user information
   - Users table: User accounts and authentication data
   - User Queries table: Historical questions and responses
   - User Selected Text table: User-selected text with context
   - Chapter Metadata table: Book structure information

### Frontend Components

1. **Chat Interface** (`frontend/index.html`, `frontend/script.js`): Main chat UI with streaming responses
2. **Text Selector** (`frontend/components/text-selector.js`): Tool for selecting text in book content
3. **Citation Display** (`frontend/components/citation-display.js`): Shows source references for responses

## Data Flow

### Ingestion Flow
1. Book files → Text extraction → Cleaning → Chunking → Embedding → Vector storage in Qdrant
2. Chapter metadata extraction → Storage in PostgreSQL

### Query Flow
1. User question → Vectorization → Similarity search in Qdrant → Context retrieval → Answer generation via Gemini → Response streaming

### Selected Text Flow
1. User selects text in frontend → Text passed to backend → RAG with restricted context → Answer generation → Response with source attribution

## Technology Stack

### Backend
- **Framework**: FastAPI
- **Database**: PostgreSQL (Neon) for metadata, Qdrant for vector storage
- **Authentication**: JWT tokens with bcrypt password hashing
- **Embeddings**: Gemini or Cohere APIs
- **Language Model**: Gemini 2.5 Flash
- **Containerization**: Docker and Docker Compose

### Frontend
- **Framework**: ChatKit-JS for chat interface
- **Language**: JavaScript/HTML/CSS
- **Styling**: Responsive CSS for mobile and desktop

### Infrastructure
- **Backend Hosting**: Railway or Render
- **Frontend Hosting**: Vercel
- **Vector Database**: Qdrant Cloud
- **Relational Database**: Neon Postgres

## Security Considerations

1. **Authentication**: JWT-based authentication with role-based access control
2. **Authorization**: Different permissions for admin, moderator, registered user, and guest
3. **Input Validation**: All API inputs are validated and sanitized
4. **Secret Management**: Environment variables for API keys and sensitive data
5. **Rate Limiting**: Protection against API abuse

## Performance & Scalability

1. **Response Time**: Target <2 seconds average response time (with <1 second for 95th percentile)
2. **Concurrency**: Support for 1000+ concurrent users
3. **Caching**: Implementation of caching for frequently asked questions
4. **Horizontal Scaling**: Architecture designed for horizontal scaling

## Error Handling & Resilience

1. **Fallback Strategies**: Multiple fallback mechanisms for external service failures
2. **Graceful Degradation**: System continues to function with reduced capabilities when external services are unavailable
3. **Circuit Breakers**: Protection against cascading failures
4. **Monitoring**: Comprehensive logging and metrics collection

## Deployment Architecture

The application is designed for containerized deployment using Docker Compose for local development and cloud platforms for production:

- **Backend**: Deployed to Railway or Render
- **Frontend**: Deployed to Vercel
- **Database**: Neon Postgres
- **Vector DB**: Qdrant Cloud
- **CI/CD**: Automated testing and deployment pipeline