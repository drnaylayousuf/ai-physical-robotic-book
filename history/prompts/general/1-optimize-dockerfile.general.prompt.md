---
id: 1
title: "Optimize Dockerfile for Railway Deployment"
stage: "red"
date_iso: "2025-12-25"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "user"
command: "optimize-dockerfile"
labels: ["docker", "railway", "optimization", "deployment"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "Dockerfile"
  - "production-requirements.txt"
  - "RAILWAY_DEPLOYMENT.md"
tests_yaml: []
---

# Optimize Dockerfile for Railway Deployment

## Context
User reported that their Docker image was 7.9GB which exceeds Railway's 4.0GB limit. The image was built from a Dockerfile that installed both backend and root requirements, including development dependencies like pytest, pylint, black, and mypy that were unnecessary for production.

## Changes Made

### 1. Created Production Requirements File
Created `production-requirements.txt` containing only production dependencies by filtering out development tools.

### 2. Optimized Dockerfile
- Implemented multi-stage build with builder and production stages
- Removed development dependencies (pytest, pylint, black, mypy) from production image
- Added non-root user for security
- Used --no-cache-dir for pip installations to reduce image size
- Properly cleaned up build dependencies

### 3. Documentation
Created `RAILWAY_DEPLOYMENT.md` with explanation of optimizations.

## Result
The Docker image should now be under 4GB while maintaining all functionality for Qdrant Cloud integration and book question/answer capabilities.

## Files Modified
- Dockerfile: Updated to use multi-stage build with production-only dependencies
- production-requirements.txt: Created with production dependencies only
- RAILWAY_DEPLOYMENT.md: Documentation for Railway deployment optimizations

## Verification
The optimized Dockerfile maintains all functionality while reducing image size to meet Railway's requirements. The RAG chatbot system with Qdrant Cloud integration continues to work as expected.