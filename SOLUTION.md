# Qdrant Connection Fix for Humanoid Robotics Book Chatbot

## Problem Identified
The issue was that the RAG model in `backend/models/rag.py` was using an in-memory store instead of connecting to the actual Qdrant vector database, even though the infrastructure was properly configured.

## Solution Implemented

### 1. Updated the RAG Model (`backend/models/rag.py`)
- Added proper Qdrant client imports
- Modified the `__init__` method to initialize a Qdrant client connection instead of using an in-memory dictionary
- Updated the `store_chunks` method to use Qdrant's upsert functionality
- Updated the `retrieve_context` method to use Qdrant's search functionality
- Added proper error handling for Qdrant connection issues
- Updated the `ingest_content` method to handle potential Qdrant errors

### 2. Key Changes Made

#### Imports:
```python
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct, Filter, FieldCondition, MatchValue
from fastapi import HTTPException
```

#### Constructor:
```python
def __init__(self):
    self.embedding_service = EmbeddingService()
    # Initialize Qdrant client
    try:
        # Check if we have an API key for authentication
        if settings.QDRANT_API_KEY:
            self.qdrant_client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                timeout=10
            )
        else:
            self.qdrant_client = QdrantClient(
                url=settings.QDRANT_URL,
                timeout=10
            )
        # Test the connection
        self.qdrant_client.get_collections()
        logger.info(f"Successfully connected to Qdrant at {settings.QDRANT_URL}")
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant at {settings.QDRANT_URL}: {e}")
        raise ConnectionError(f"Could not connect to Qdrant: {e}")
```

#### Store Chunks Method:
```python
async def store_chunks(self, chunks: List[str], embeddings: List[List[float]], collection_name: str = None) -> List[str]:
    if collection_name is None:
        collection_name = settings.QDRANT_COLLECTION_NAME

    # Check if collection exists, create if it doesn't
    try:
        self.qdrant_client.get_collection(collection_name)
    except:
        # Create collection if it doesn't exist
        self.qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=len(embeddings[0]) if embeddings else 384, distance=models.Distance.COSINE)
        )
        logger.info(f"Created new Qdrant collection: {collection_name}")

    # Prepare points for insertion
    points = []
    chunk_ids = []
    for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        chunk_id = f"{collection_name}_chunk_{len(chunk_ids)}"
        chunk_ids.append(chunk_id)

        points.append(
            PointStruct(
                id=chunk_id,
                vector=embedding,
                payload={
                    "content": chunk,
                    "collection_name": collection_name,
                    "chunk_index": i
                }
            )
        )

    # Insert points into Qdrant
    self.qdrant_client.upsert(
        collection_name=collection_name,
        points=points
    )

    logger.info(f"Stored {len(chunk_ids)} chunks in collection {collection_name}")
    return chunk_ids
```

#### Retrieve Context Method:
```python
async def retrieve_context(self, query: str, top_k: int = 5, collection_name: str = None) -> List[Dict]:
    if collection_name is None:
        collection_name = settings.QDRANT_COLLECTION_NAME

    # Generate embedding for the query
    query_embedding = await self.embedding_service.generate_embedding(query)

    # Search in Qdrant
    try:
        search_results = self.qdrant_client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True
        )
    except Exception as e:
        logger.error(f"Error searching in Qdrant: {e}")
        return []

    results = []
    for result in search_results:
        results.append({
            "chunk_id": result.id,
            "content": result.payload.get("content", ""),
            "confidence": result.score
        })

    return results
```

### 3. Requirements Update
Updated `requirements.txt` to use a more recent version of qdrant-client:
```
qdrant-client>=1.10.0
```

## How to Deploy the Fix

### Option 1: Using Docker (Recommended)
1. Make sure Docker and Docker Compose are installed
2. Start the services: `docker-compose up -d`
3. The Qdrant service will be available at `http://localhost:6333`
4. The backend will connect to Qdrant automatically

### Option 2: Running Qdrant Separately
1. Start Qdrant with Docker: `docker run -p 6333:6333 qdrant/qdrant`
2. Run the backend service: `python -m backend.main`

### Option 3: Using External Qdrant Service
1. Update the `.env` file with your Qdrant configuration:
   ```
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_api_key_if_needed
   QDRANT_COLLECTION_NAME=your_collection_name
   ```
2. Run the application normally

## Testing the Fix

To verify the connection is working:

1. Make sure Qdrant is running
2. The application should start without connection errors
3. When you ask questions to the chatbot, it should now properly query the Qdrant vector database
4. Check the logs for successful Qdrant connection messages

## Troubleshooting Common Issues

1. **"Failed to fetch" error**: This usually means the backend can't connect to Qdrant
   - Verify Qdrant service is running
   - Check that the URL in settings matches the Qdrant service URL
   - If using Docker, make sure both services are on the same network

2. **"No connection in Qdrant" error**:
   - Ensure the Qdrant service is accessible at the configured URL
   - Check firewall settings if Qdrant is on a different machine
   - Verify API keys if authentication is required

3. **Collection not found**: The system will automatically create the collection if it doesn't exist

This fix resolves the "Failed to fetch" and "no connection in qdrant" errors by properly connecting the RAG model to the Qdrant vector database instead of using an in-memory store.