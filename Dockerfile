# Simple Dockerfile that installs packages directly in final stage
FROM python:3.11-slim

WORKDIR /app

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gcc \
    && rm -rf /var/lib/apt/lists/*

# Copy the production requirements file
COPY production-requirements.txt .

# Install Python dependencies directly
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir --upgrade setuptools && \
    pip install --no-cache-dir -r production-requirements.txt && \
    # Verify uvicorn is installed and accessible
    python -c "import uvicorn; print('uvicorn version:', uvicorn.__version__)" && \
    # Clean up pip cache and temporary files after installation
    rm -rf /root/.cache/pip && \
    rm -rf /tmp/*

# Copy application code
COPY backend/ ./backend/
COPY start_backend.py ./

# Remove unnecessary files that are not needed in production
RUN rm -rf ./backend/qdrant_storage/ ./backend/rag_chatbot.db ./backend/.env ./backend/.pytest_cache/ ./backend/__pycache__/ ./backend/migrations/ ./backend/docs/

# Set environment variables
ENV PYTHONPATH=/app
ENV HOST=0.0.0.0
ENV PORT=8000
ENV BOOK_CONTENT_PATH=./doc

# Expose the port
EXPOSE 8000

# Run the FastAPI application using the dedicated Python script
CMD ["python", "backend/run_server.py"]