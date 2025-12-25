#!/bin/sh
# Shell script to start the server with proper environment variable handling

# Set default port if not provided
PORT=${PORT:-8000}

echo "Starting server on port $PORT"

# Validate that PORT is a number
if ! [ "$PORT" -eq "$PORT" ] 2>/dev/null; then
    echo "Error: PORT '$PORT' is not a valid integer. Using default port 8000."
    PORT=8000
fi

# Run the Python server script with the validated port
exec python backend/run_server.py