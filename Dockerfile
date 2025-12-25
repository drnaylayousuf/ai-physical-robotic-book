# Multi-stage build to reduce image size
FROM python:3.11-slim as builder

WORKDIR /app

# Install build dependencies only when needed
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy the production requirements file
COPY production-requirements.txt .

# Install Python dependencies for production only
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r production-requirements.txt

# Production stage
FROM python:3.11-slim

WORKDIR /app

# Create non-root user for security
RUN groupadd -r appuser && useradd -r -g appuser appuser

# Install only essential runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

# Copy installed Python packages from builder stage
COPY --from=builder /usr/local/lib/python3.11/site-packages /usr/local/lib/python3.11/site-packages

# Copy application code
COPY backend/ ./backend/
COPY start_backend.py ./

# Change ownership to non-root user
RUN chown -R appuser:appuser /app
USER appuser

# Set environment variables
ENV PYTHONPATH=/app
ENV HOST=0.0.0.0
ENV PORT=8000
ENV BOOK_CONTENT_PATH=./doc

# Expose the port
EXPOSE 8000

# Run the FastAPI application
CMD ["uvicorn", "backend.main:app", "--host", "0.0.0.0", "--port", "8000", "--reload=false"]