#!/usr/bin/env python3
"""
Diagnostic script to check Qdrant collection status and content for troubleshooting.
This script connects to Qdrant, prints collection names, number of chunks, and first 3 chunks from the collection.
"""

from qdrant_client import QdrantClient
from qdrant_client.http import models
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend'))

from backend.config.settings import settings
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def run_diagnostic():
    """Run the diagnostic check for Qdrant collection status and content."""
    try:
        # Initialize Qdrant client
        if settings.QDRANT_API_KEY:
            client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                timeout=10
            )
        else:
            client = QdrantClient(
                url=settings.QDRANT_URL,
                timeout=10
            )

        logger.info(f"Successfully connected to Qdrant at {settings.QDRANT_URL}")

        # List collections
        collections = client.get_collections()
        collection_names = [c.name for c in collections.collections]
        print("Available collections:", collection_names)

        # Get collection info for the book chunks
        collection_name = settings.QDRANT_COLLECTION_NAME
        try:
            collection_info = client.get_collection(collection_name)
            print(f"Collection '{collection_name}' has {collection_info.points_count} chunks")

            # Get first 3 chunks as sample
            scroll_result = client.scroll(
                collection_name=collection_name,
                limit=3,
                with_payload=True,
                with_vectors=False
            )

            print("Sample chunks:")
            for i, point in enumerate(scroll_result[0]):
                print(f"  {i+1}. ID: {point.id}")
                print(f"     Content: {point.payload.get('content', '')[:100]}...")
                print(f"     Source: {point.payload.get('source', point.payload.get('collection_name', 'Unknown'))}")
                print("     ---")

        except Exception as e:
            print(f"Collection '{collection_name}' does not exist or is empty: {e}")
            print("This is expected if no content has been ingested yet.")

        print("\nDiagnostic completed successfully!")
        return {
            "collections": collection_names,
            "total_chunks": collection_info.points_count if 'collection_info' in locals() else 0,
            "sample_chunks": [{"id": point.id, "content": point.payload.get('content', '')[:100], "source": point.payload.get('source', point.payload.get('collection_name', 'Unknown'))} for point in scroll_result[0]] if 'scroll_result' in locals() else [],
            "status": "healthy"
        }

    except Exception as e:
        logger.error(f"Diagnostic check failed: {e}")
        print(f"Diagnostic check failed: {e}")
        return {
            "collections": [],
            "total_chunks": 0,
            "sample_chunks": [],
            "status": "failed",
            "error": str(e)
        }

if __name__ == "__main__":
    result = run_diagnostic()
    print(f"\nResult: {result}")