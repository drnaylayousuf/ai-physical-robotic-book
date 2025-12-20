# Quickstart: Fix Chatbot UI Configuration Issue

## Problem
The chatbot UI returns "The book does not provide details about this topic. No context is available and no generative model is configured." when users ask questions.

## Root Cause
The frontend UI is configured to call port 8000, but the backend with properly ingested content runs on port 8001.

## Solution
Run the ingestion script to set up the backend on port 8000 with proper content.

### Step 1: Run the Ingestion Script
```bash
# Navigate to the project root directory
cd C:\Users\nayla\OneDrive\Desktop\humanoid-robotics-book

# Run the restart and ingestion script
./restart_and_ingest.bat
```

This script will:
1. Stop any existing Python processes
2. Start the backend server on port 8000
3. Wait for the server to be ready
4. Generate an admin token
5. Call the ingestion endpoint to process book content from the `doc/` directory
6. Verify the ingestion with diagnostic and QA tests

### Step 2: Verify the Fix
After the script completes successfully:

1. **Check that the server is running on port 8000**:
   ```bash
   curl http://localhost:8000/api/health
   ```

2. **Test the diagnostic endpoint**:
   ```bash
   curl http://localhost:8000/api/diagnostic/qdrant
   ```
   This should show a non-zero chunk count indicating content has been ingested.

3. **Test the QA functionality**:
   ```bash
   curl -X POST http://localhost:8000/api/ask \
     -H "Content-Type: application/json" \
     -d '{"question": "What is humanoid robotics?", "mode": "full_book"}'
   ```

### Step 3: Test the UI
1. Start your Docusaurus book site (if not already running)
2. Open the book in your browser
3. Use the chatbot UI to ask questions about the book content
4. You should now receive meaningful responses instead of the error message

## Alternative Approach (if the above doesn't work)
If the restart_and_ingest.bat script doesn't resolve the issue, you can alternatively:

1. **Manually start the backend on port 8000**:
   ```bash
   cd backend
   uvicorn main:app --host 0.0.0.0 --port 8000 --reload
   ```

2. **Manually run the ingestion process**:
   - First, ensure you have an admin account and token
   - Call the `/api/ingest` endpoint with proper authentication
   - Verify content is ingested using the diagnostic endpoint

## Troubleshooting
- If the ingestion fails, verify that the `doc/` directory contains book content
- Ensure your `.env` file has the required API keys (QDRANT_API_KEY, GEMINI_API_KEY)
- Check that Qdrant is accessible and properly configured
- Verify that the database connection is working

## Files Modified/Affected
- `frontend/components/EmbeddedChatbot.jsx` - UI API endpoint (no changes needed)
- `frontend/hooks/useChatbotAPI.js` - API hook (no changes needed)
- `backend/main.py` - Backend server (no changes needed)
- `backend/models/rag.py` - RAG model (no changes needed)

## Expected Outcome
After running the script, the chatbot UI should return meaningful responses to user questions instead of the error message, as the backend on port 8000 will have both ingested content and properly configured generative model.