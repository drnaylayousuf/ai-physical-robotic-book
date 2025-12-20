# Quickstart: Qdrant Cloud Migration

## Prerequisites
- Python 3.11+
- Qdrant Cloud account with API key
- Cohere API key
- Environment variables configured:
  - QDRANT_URL=https://<your-cloud-url>.qdrant.io
  - QDRANT_API_KEY=qdrant_<your-api-key>
  - COHERE_API_KEY=<your-cohere-api-key>

## Setup
1. Install required dependencies:
   ```bash
   pip install qdrant-client cohere
   ```

2. Configure environment variables in your `.env` file

3. Verify Qdrant Cloud connection:
   ```bash
   curl -H "api-key: $QDRANT_API_KEY" $QDRANT_URL/health
   ```

## Migration Process
1. Run the migration script to upload existing book chunks:
   ```bash
   python migrate_to_qdrant.py
   ```

2. The script will:
   - Connect to Qdrant Cloud
   - Check for existing chunks
   - Generate Cohere embeddings for missing chunks
   - Upload chunks with metadata to Qdrant Cloud
   - Report migration statistics

## Usage
1. Start the backend service:
   ```bash
   python -m backend.main
   ```

2. The chat API will now use Qdrant Cloud for vector search
3. All book content queries will be served from Qdrant Cloud

## Verification
1. Test the health endpoint: `GET /health`
2. Submit a test query to verify Qdrant integration
3. Check logs for Qdrant Cloud connection status