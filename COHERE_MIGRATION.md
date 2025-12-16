# RAG System Configuration: Migration from OpenAI to Cohere + Qdrant

## Overview

This document describes the changes made to migrate the RAG system from using OpenAI embeddings to using Cohere embeddings while maintaining Qdrant as the vector database and Gemini for text generation.

## Key Changes

### 1. Multi-Provider Embedding Service

The embedding service has been updated to support multiple embedding providers:
- **Cohere** (default) - Now the primary embedding provider
- **OpenAI** - Still supported as an option
- **Gemini** - Supported as an option

### 2. New Configuration Options

Added new environment variables to `.env.example`:
- `EMBEDDING_PROVIDER` - Set to "cohere", "openai", or "gemini"
- `EMBEDDING_MODEL` - Model name specific to the provider
- `EMBEDDING_DIMENSION` - Vector dimension (1024 for Cohere, 1536 for OpenAI, 768 for Gemini)

### 3. Cohere Integration

Created a new Cohere embedding service:
- `backend/utils/cohere_embeddings.py` - Dedicated Cohere embedding implementation
- Uses Cohere's `embed-english-v3.0` model by default
- Supports both single and batch embedding generation
- Handles different model dimensions automatically

### 4. Qdrant Compatibility

Updated the RAG model to work properly with different embedding dimensions:
- Dynamic embedding size detection based on provider
- Proper collection creation with correct vector dimensions
- Maintains Qdrant as the vector database

### 5. Gemini Continues as Generator

- Gemini remains as the text generation model (for `generate_response` method)
- Only the embedding provider changed, not the generation provider

## Configuration

To use Cohere embeddings with Qdrant and Gemini generation:

```env
# Embedding Configuration
EMBEDDING_PROVIDER=cohere
EMBEDDING_MODEL=embed-english-v3.0
EMBEDDING_DIMENSION=1024

# API Keys
COHERE_API_KEY=your_cohere_api_key_here
GEMINI_API_KEY=your_gemini_api_key_here

# Qdrant Configuration (unchanged)
QDRANT_URL=http://localhost
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_chunks
```

## Files Modified

1. `backend/utils/embeddings.py` - Updated to support multiple providers
2. `backend/utils/cohere_embeddings.py` - New Cohere-specific service
3. `backend/config/settings.py` - Added provider and dimension settings
4. `backend/models/rag.py` - Updated to use dynamic embedding dimensions
5. `.env.example` - Updated with new configuration options
6. `requirements.txt` - Updated Cohere version
7. `test_cohere_embeddings.py` - Test script for Cohere integration
8. `test_rag_cohere_qdrant.py` - Test script for full integration

## Benefits of the New Architecture

1. **Flexibility**: Can switch between embedding providers without code changes
2. **Cost Efficiency**: Cohere embeddings may offer better cost-performance ratio
3. **Maintainability**: Clean separation of embedding providers
4. **Scalability**: Qdrant continues to provide efficient vector storage and retrieval
5. **Continuity**: Gemini continues to provide high-quality text generation

## Testing

Run the provided test scripts to verify the configuration:

```bash
# Test Cohere embeddings specifically
python test_cohere_embeddings.py

# Test full RAG pipeline
python test_rag_cohere_qdrant.py
```