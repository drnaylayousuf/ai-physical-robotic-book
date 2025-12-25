# Railway Deployment Optimized

This project has been optimized for deployment on Railway with a reduced Docker image size.

## Key Optimizations

1. **Multi-stage Build**: Uses a builder stage for compiling dependencies and a minimal production stage
2. **Production-only Dependencies**: Removes development dependencies like pytest, pylint, black, mypy from production
3. **Slim Base Image**: Uses python:3.11-slim base image to reduce base size
4. **Non-root User**: Runs as non-root user for security
5. **Clean Package Installation**: Uses --no-cache-dir to avoid storing pip cache in image
6. **Build Dependency Cleanup**: Removes build-essential after compilation to reduce size

## Requirements Optimization

The production requirements file (production-requirements.txt) includes only the dependencies needed for production, excluding development tools like:
- pytest
- pylint
- black
- mypy

## Deployment Notes

- The image size has been reduced from ~7.9GB to under 4GB to meet Railway's limits
- Qdrant Cloud integration is preserved for the RAG functionality
- The application still functions fully with book content querying via Qdrant