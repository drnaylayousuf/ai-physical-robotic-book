# Multi-stage Dockerfile to reduce image size
FROM python:3.11-slim as builder

WORKDIR /app

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gcc \
    libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy the production requirements file
COPY production-requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir --upgrade setuptools && \
    pip install --no-cache-dir -r production-requirements.txt

# Production stage
FROM python:3.11-slim

WORKDIR /app

# Install only runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy installed Python packages from builder stage
COPY --from=builder /usr/local/lib/python3.11/site-packages /usr/local/lib/python3.11/site-packages

# Copy application code
COPY backend/ ./backend/
COPY start_backend.py ./
COPY start_server.sh ./
COPY start_app.py ./

# Remove unnecessary files that are not needed in production
RUN rm -rf ./backend/qdrant_storage/ ./backend/rag_chatbot.db ./backend/.env ./backend/.pytest_cache/ ./backend/__pycache__/ ./backend/migrations/ ./backend/docs/

# Set environment variables - DO NOT set PORT here to avoid conflicts
ENV PYTHONPATH=/app
ENV HOST=0.0.0.0
ENV BOOK_CONTENT_PATH=./doc

# Expose the port
EXPOSE 8000

# Run the FastAPI application using direct Python execution
CMD ["python", "start_app.py"]