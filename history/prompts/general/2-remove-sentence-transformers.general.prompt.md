---
id: 2
title: "Remove sentence-transformers package to reduce Docker size"
stage: "red"
date_iso: "2025-12-25"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "user"
command: "remove-sentence-transformers"
labels: ["docker", "optimization", "dependencies", "size"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "production-requirements.txt"
  - "Dockerfile"
tests_yaml: []
---

# Remove sentence-transformers package to reduce Docker size

## Context
After initial Docker optimization, the image was still too large for Railway deployment. Investigation revealed that the sentence-transformers package was included in production requirements but not actually used in the code, since the application uses Cohere for embeddings.

## Changes Made

### 1. Removed unused dependency
- Removed `sentence-transformers==5.1.2` from production-requirements.txt
- This package was adding hundreds of MB to the image with ML models that weren't needed

### 2. Enhanced Dockerfile cleanup
- Added removal of unnecessary backend files (qdrant_storage, rag_chatbot.db, cache directories)
- Improved cleanup of temporary files and caches

## Result
The Docker image size should be significantly reduced by removing the large sentence-transformers package while maintaining all functionality for Qdrant Cloud integration and book question/answer capabilities.

## Files Modified
- production-requirements.txt: Removed sentence-transformers dependency
- Dockerfile: Enhanced cleanup of unnecessary files

## Verification
The application maintains all functionality while significantly reducing image size for Railway deployment.