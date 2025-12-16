---
id: 002
title: migrate-rag-system-from-openai-to-cohere-embeddings
stage: refactor
date: '2025-12-11'
model: claude-sonnet-4-5-20250929
feature: main
branch: 001-fix-rag-workflow
user: nayla
command: sp.specify i want you to make open ai remove from my rag system from it and apply gemini which previusly was working and make my rag with qdrant with cohere and and gemini not open ai
labels: ["rag", "cohere", "qdrant", "migration", "embeddings"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - backend/utils/cohere_embeddings.py
  - backend/utils/embeddings.py
  - backend/config/settings.py
  - backend/models/rag.py
  - .env.example
  - requirements.txt
  - test_cohere_embeddings.py
  - test_rag_cohere_qdrant.py
  - COHERE_MIGRATION.md
tests: []
---

# Migrate RAG System from OpenAI to Cohere Embeddings

## PROMPT_TEXT

sp.specify i want you to make open ai remove from my rag system from it and apply gemini which previusly was working and make my rag with qdrant with cohere and and gemini not open ai

## RESPONSE_TEXT

Successfully modified the RAG system to replace OpenAI with Cohere embeddings while maintaining Qdrant as the vector database and keeping Gemini for text generation. Key changes included:

1. Created a new Cohere embedding service (backend/utils/cohere_embeddings.py) that handles embedding generation using Cohere's API
2. Updated the main embedding service (backend/utils/embeddings.py) to support multiple providers (Cohere, OpenAI, Gemini) with a single interface
3. Updated configuration settings (backend/config/settings.py) to include EMBEDDING_PROVIDER, EMBEDDING_MODEL, and EMBEDDING_DIMENSION
4. Updated the environment example file (.env.example) with new configuration options
5. Enhanced the RAG model (backend/models/rag.py) to work with dynamic embedding dimensions from different providers
6. Updated dependencies in requirements.txt to ensure proper Cohere library version
7. Created test scripts (test_cohere_embeddings.py and test_rag_cohere_qdrant.py) to verify the new Cohere integration
8. Added comprehensive documentation in COHERE_MIGRATION.md explaining all changes

The system now uses Cohere for embeddings with Qdrant as the vector database and Gemini for text generation, with the ability to switch between embedding providers via configuration.

## OUTCOME

The RAG system has been successfully migrated from OpenAI embeddings to Cohere embeddings while maintaining Qdrant as the vector database and Gemini for text generation. The architecture is now more flexible, supporting multiple embedding providers through configuration changes.