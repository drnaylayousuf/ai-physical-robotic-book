#!/usr/bin/env python3
"""
Direct test of Qdrant connection and content retrieval.
"""

from qdrant_client import QdrantClient
from backend.config.settings import settings
import os

def test_qdrant_connection():
    """Test direct Qdrant connection."""
    print("Testing Qdrant connection...")

    try:
        # Try to connect to local Qdrant storage
        print(f"Connecting to local Qdrant at: {settings.QDRANT_LOCAL_PATH}")

        # Check if the path exists
        if not os.path.exists(settings.QDRANT_LOCAL_PATH):
            print(f"Qdrant local path does not exist: {settings.QDRANT_LOCAL_PATH}")
            return False

        # Connect to local Qdrant
        client = QdrantClient(path=settings.QDRANT_LOCAL_PATH)
        print("Successfully connected to local Qdrant!")

        # List collections
        collections = client.get_collections()
        print(f"Collections: {[c.name for c in collections.collections]}")

        # Check the book_chunks collection specifically
        try:
            collection_info = client.get_collection(settings.QDRANT_COLLECTION_NAME)
            print(f"Collection '{settings.QDRANT_COLLECTION_NAME}' exists with {collection_info.points_count} points")

            # Try to get some points to verify content exists
            if collection_info.points_count > 0:
                # Get first few points
                scroll_result = client.scroll(
                    collection_name=settings.QDRANT_COLLECTION_NAME,
                    limit=3,
                    with_payload=True,
                    with_vectors=False
                )

                print(f"Sample points retrieved: {len(scroll_result[0])}")
                for i, point in enumerate(scroll_result[0]):
                    print(f"  Point {i+1}: ID={point.id}, Content preview: {point.payload.get('content', '')[:100]}...")
            else:
                print("No points in the collection")

        except Exception as e:
            print(f"Collection '{settings.QDRANT_COLLECTION_NAME}' does not exist: {e}")

        return True

    except Exception as e:
        print(f"Failed to connect to Qdrant: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_qdrant_connection()
    if success:
        print("\nQdrant connection test PASSED")
    else:
        print("\nQdrant connection test FAILED")