# Data Model: Chatbot UI Configuration Issue

## Overview
This data model addresses the configuration issue where the chatbot UI returns "The book does not provide details about this topic. No context is available and no generative model is configured." due to a mismatch between frontend and backend configurations.

## System Components

### 1. Backend Service Configuration
- **Port**: 8000 (UI-facing) / 8001 (test/working backend)
- **API Endpoints**: `/api/ask`, `/api/ingest`, `/api/health`
- **Environment**: Must have proper `.env` configuration with GEMINI_API_KEY
- **Content Status**: Book content must be ingested into vector database

### 2. Frontend UI Configuration
- **API Base URL**: `http://localhost:8000/api` (currently hardcoded)
- **Request Format**: JSON with question, mode, and optional selected_text
- **Response Handling**: Processes RAG responses with sources and references

### 3. Vector Database (Qdrant)
- **Collection**: `book_chunks` containing embedded book content
- **Status**: Must be populated with content for proper RAG functionality
- **Connection**: Configured via QDRANT_URL and QDRANT_API_KEY in environment

### 4. Generative Model Configuration
- **Provider**: Google Gemini 2.5 Flash
- **API Key**: GEMINI_API_KEY from environment
- **Status**: Must be initialized and accessible to the RAG pipeline

## Data Flow

### Current Broken Flow
1. UI sends request to `http://localhost:8000/api/ask`
2. Backend on port 8000 receives request
3. Backend has no ingested content or uninitialized generative model
4. RAG pipeline returns fallback error message

### Expected Fixed Flow
1. UI sends request to `http://localhost:8000/api/ask`
2. Backend on port 8000 receives request
3. Backend has properly ingested content and initialized generative model
4. RAG pipeline retrieves context and generates response
5. UI receives and displays meaningful response

## Configuration Requirements

### Environment Variables
- `QDRANT_URL`: Vector database connection
- `QDRANT_API_KEY`: Vector database authentication
- `GEMINI_API_KEY`: Generative model API key
- `QDRANT_COLLECTION_NAME`: Collection containing book chunks

### Content Ingestion State
- Book content must be processed and stored in vector database
- Ingestion process must have completed successfully
- Diagnostic endpoint should show non-zero chunk count

## Error Conditions

### Current Error State
- No content in vector database (0 chunks)
- Generative model not initialized
- Returns: "The book does not provide details about this topic. No context is available and no generative model is configured."

### Success State
- Content properly ingested in vector database
- Generative model initialized and accessible
- Returns: Contextually relevant answers to user questions