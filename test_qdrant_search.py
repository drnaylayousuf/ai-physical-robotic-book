#!/usr/bin/env python3
"""
Test script to directly test Qdrant search functionality
"""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from qdrant_client import QdrantClient
from backend.config.settings import settings
from backend.utils.embeddings import EmbeddingService

async def test_qdrant_search():
    """Test direct Qdrant search functionality"""
    print("Testing direct Qdrant search functionality...")

    # Initialize embedding service
    embedding_service = EmbeddingService()

    # Connect to Qdrant Cloud
    client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
        timeout=30
    )

    # Test query
    query_text = "humanoid robotics"
    print(f"Searching for: '{query_text}'")

    # Generate embedding for the query
    query_embedding = await embedding_service.generate_embedding(query_text)
    print(f"Generated embedding with {len(query_embedding)} dimensions")

    # Perform search in Qdrant
    try:
        search_results = client.search(
            collection_name=settings.QDRANT_COLLECTION_NAME,
            query_vector=query_embedding,
            limit=3,
            with_payload=True
        )

        print(f"Found {len(search_results)} results:")
        for i, result in enumerate(search_results):
            content_preview = result.payload.get("content", "")[:100] + "..." if len(result.payload.get("content", "")) > 100 else result.payload.get("content", "")
            print(f"  Result {i+1} (Score: {result.score:.4f}): {content_preview}")

        if len(search_results) == 0:
            print("No results found - there may be an issue with the stored embeddings")
            return False
        else:
            print("✅ Search is working and returning results!")
            return True

    except Exception as e:
        print(f"ERROR: Error during search: {e}")
        # Try using query_points method as an alternative
        try:
            print("Trying alternative method using query_points...")
            search_results = client.query_points(
                collection_name=settings.QDRANT_COLLECTION_NAME,
                query=query_embedding,
                limit=3,
                with_payload=True
            ).points

            print(f"Found {len(search_results)} results using query_points:")
            for i, result in enumerate(search_results):
                content_preview = result.payload.get("content", "")[:100] + "..." if len(result.payload.get("content", "")) > 100 else result.payload.get("content", "")
                print(f"  Result {i+1} (Score: {result.score:.4f}): {content_preview}")

            if len(search_results) > 0:
                print("SUCCESS: Alternative search method worked!")
                return True
            else:
                print("No results found with alternative method")
                return False
        except Exception as e2:
            print(f"ERROR: Alternative search method also failed: {e2}")
            return False

if __name__ == "__main__":
    import asyncio
    success = asyncio.run(test_qdrant_search())
    if success:
        print("\nSUCCESS: Direct Qdrant search test completed successfully!")
    else:
        print("\nERROR: Direct Qdrant search test failed!")
        sys.exit(1)