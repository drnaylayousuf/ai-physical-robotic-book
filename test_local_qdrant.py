#!/usr/bin/env python3
"""
Direct test of Qdrant local storage to verify it's working properly.
"""

from qdrant_client import QdrantClient
from qdrant_client.http import models
import os

def test_local_qdrant():
    """Test local Qdrant storage directly."""

    # Use the same path as configured in settings
    qdrant_path = "./qdrant_storage"

    print(f"Testing local Qdrant storage at: {qdrant_path}")

    try:
        # Create a local Qdrant client
        client = QdrantClient(path=qdrant_path)
        print("✅ Successfully connected to local Qdrant storage")

        # List existing collections
        collections = client.get_collections()
        print(f"Existing collections: {[c.name for c in collections.collections]}")

        # Create a test collection
        collection_name = "test_collection"
        try:
            client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE)
            )
            print(f"✅ Created collection: {collection_name}")
        except Exception as e:
            print(f"ℹ️  Collection {collection_name} may already exist: {e}")

        # Add a test point
        test_point = models.PointStruct(
            id=1,
            vector=[0.1] * 1536,  # Simple test vector
            payload={"content": "This is a test document about humanoid robots.", "source": "test"}
        )

        client.upsert(
            collection_name=collection_name,
            points=[test_point]
        )
        print("✅ Added test point to collection")

        # Count points in the collection
        count = client.count(collection_name=collection_name)
        print(f"✅ Collection '{collection_name}' now has {count.count} points")

        # Query the collection
        search_result = client.search(
            collection_name=collection_name,
            query_vector=[0.1] * 1536,
            limit=1
        )

        if search_result:
            print(f"✅ Successfully retrieved {len(search_result)} results from search")
            print(f"   Content: {search_result[0].payload.get('content', 'N/A')}")
        else:
            print("⚠️  No results returned from search")

        # Check the book_chunks collection specifically
        try:
            book_chunks_count = client.count(collection_name="book_chunks")
            print(f"📊 'book_chunks' collection has {book_chunks_count.count} points")
        except Exception as e:
            print(f"ℹ️  'book_chunks' collection doesn't exist yet: {e}")

        return True

    except Exception as e:
        print(f"❌ Error testing local Qdrant: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    test_local_qdrant()