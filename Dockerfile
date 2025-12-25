# Use Python base image for the backend API
FROM python:3.11-slim

WORKDIR /app

# Copy the backend requirements
COPY backend/requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# Install additional dependencies from root if they exist
COPY requirements.txt /tmp/root-requirements.txt
RUN if [ -f /tmp/root-requirements.txt ]; then \
        pip install --no-cache-dir -r /tmp/root-requirements.txt; \
    fi

# Copy only necessary backend files
COPY backend/ ./backend/
COPY start_backend.py ./

# Set environment variables
ENV PYTHONPATH=/app
ENV HOST=0.0.0.0
ENV PORT=8000
ENV BOOK_CONTENT_PATH=./backend
# Point to a minimal directory that exists

# Expose the port
EXPOSE 8000

# Run the FastAPI application
CMD ["uvicorn", "backend.main:app", "--host", "0.0.0.0", "--port", "8000", "--reload=false"]