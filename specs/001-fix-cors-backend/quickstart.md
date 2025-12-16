# Quickstart: Fix CORS Communication Between Frontend and Backend

## Overview
This guide will help you fix the CORS (Cross-Origin Resource Sharing) issue preventing communication between your frontend (localhost:3000) and backend (localhost:8000/api/ask).

## Prerequisites
- FastAPI backend running locally
- Frontend application running locally
- Python environment with FastAPI installed

## Step 1: Add CORS Middleware to FastAPI Backend

1. Open your main FastAPI application file (typically `main.py` or `app.py`)

2. Add the following import at the top of the file:
   ```python
   from fastapi.middleware.cors import CORSMiddleware
   ```

3. Add the CORS middleware configuration after creating your FastAPI app instance:
   ```python
   app = FastAPI()

   # Add CORS middleware
   app.add_middleware(
       CORSMiddleware,
       allow_origins=["http://localhost:3000"],  # Frontend URL
       allow_credentials=True,
       allow_methods=["*"],  # Allows all methods (GET, POST, OPTIONS, etc.)
       allow_headers=["*"],  # Allows all headers
   )
   ```

## Step 2: Verify Frontend API Configuration

1. Check your frontend API service file (e.g., `api.js`, `services/api.js`, etc.)
2. Ensure the backend URL is correctly set to `http://localhost:8000`
3. Example configuration:
   ```javascript
   const BACKEND_URL = 'http://localhost:8000';
   const API_ENDPOINT = `${BACKEND_URL}/api/ask`;
   ```

## Step 3: Restart the Backend Server

1. Stop the running backend server (Ctrl+C)
2. Restart the server using your preferred method:
   - Using uvicorn: `uvicorn main:app --reload --port 8000`
   - Using python: `python main.py`
   - Using your project's start script

## Step 4: Test the Communication

1. Start your frontend application (if not already running)
2. Try submitting a question through the frontend
3. Check browser developer tools for any CORS errors in the Console and Network tabs
4. Verify that the response is received successfully from the backend

## Troubleshooting

### If CORS errors persist:
- Double-check that the middleware is added to your FastAPI app
- Verify the origin in `allow_origins` exactly matches your frontend URL
- Check that you restarted the backend after making changes
- Look for typos in the configuration

### Common Origin Mismatches:
- `http://localhost:3000` vs `http://127.0.0.1:3000`
- `https://localhost:3000` vs `http://localhost:3000`
- `http://localhost:3000/` vs `http://localhost:3000`

## Production Considerations

For production deployment:
- Replace `["http://localhost:3000"]` with your actual production frontend URL
- Be more restrictive with `allow_methods` and `allow_headers` if possible
- Consider using environment variables for origin configuration