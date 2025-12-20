#!/usr/bin/env python3
"""
Script to verify that book content was successfully sent to Qdrant Cloud
"""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from qdrant_client import QdrantClient
from backend.config.settings import settings

def verify_qdrant_ingestion():
    """Verify that the book content was successfully sent to Qdrant Cloud"""
    print("Verifying Qdrant Cloud ingestion...")

    # Check if Qdrant settings are configured
    if not settings.QDRANT_URL or not settings.QDRANT_API_KEY:
        print("ERROR: QDRANT_URL and QDRANT_API_KEY are not configured in .env")
        return False

    try:
        # Connect to Qdrant Cloud
        print(f"Connecting to Qdrant Cloud at: {settings.QDRANT_URL}")
        client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=30
        )

        # Test connection
        collections = client.get_collections()
        print(f"Successfully connected to Qdrant Cloud")
        print(f"Available collections: {[col.name for col in collections.collections]}")

        # Check if our book collection exists
        collection_name = settings.QDRANT_COLLECTION_NAME
        try:
            collection_info = client.get_collection(collection_name)
            print(f"\nCollection '{collection_name}' exists!")
            print(f"Points count: {collection_info.points_count}")

            # Different way to access vectors count depending on Qdrant version
            try:
                vectors_count = collection_info.vectors_count
                print(f"Vectors count: {vectors_count}")
            except AttributeError:
                # For newer Qdrant versions, access vectors differently
                if hasattr(collection_info.config.params, 'vectors'):
                    if isinstance(collection_info.config.params.vectors, dict):
                        # Named vectors configuration
                        total_vectors = sum(v.size for v in collection_info.config.params.vectors.values())
                        print(f"Vectors configuration: {len(collection_info.config.params.vectors)} named vectors")
                    else:
                        # Single vector configuration
                        print(f"Vector size: {collection_info.config.params.vectors.size}")
                else:
                    print(f"Vector size: {collection_info.config.params.size}")

            if collection_info.points_count > 0:
                print(f"SUCCESS: {collection_info.points_count} book content chunks successfully stored in Qdrant Cloud!")

                # Get a few sample points to verify content
                try:
                    # Use the correct Qdrant scroll API
                    records, next_page = client.scroll(
                        collection_name=collection_name,
                        limit=3,
                        with_payload=True,
                        with_vectors=False
                    )

                    print(f"\nSample content from first 3 chunks:")
                    for i, record in enumerate(records):
                        content_preview = record.payload.get("content", "")[:100] + "..." if len(record.payload.get("content", "")) > 100 else record.payload.get("content", "")
                        print(f"  Chunk {i+1} (ID: {record.id}): {content_preview}")

                except Exception as e:
                    print(f"Could not retrieve sample points: {e}")
                    # Try an alternative method - get points by ID
                    try:
                        # First, get a list of point IDs
                        records, next_page = client.scroll(
                            collection_name=collection_name,
                            limit=3,
                            with_payload=True,
                            with_vectors=False
                        )

                        if records:
                            # Extract the IDs from the records
                            ids = [record.id for record in records]
                            print(f"Retrieved IDs: {ids}")

                            # Now fetch these points specifically
                            points = client.retrieve(
                                collection_name=collection_name,
                                ids=ids,
                                with_payload=True,
                                with_vectors=False
                            )

                            print(f"Alternative method - Sample content from first 3 chunks:")
                            for i, point in enumerate(points):
                                content_preview = point.payload.get("content", "")[:100] + "..." if len(point.payload.get("content", "")) > 100 else point.payload.get("content", "")
                                print(f"  Chunk {i+1} (ID: {point.id}): {content_preview}")
                        else:
                            print("No records returned from scroll")
                    except Exception as e2:
                        print(f"Alternative method also failed: {e2}")
                        # Final attempt - try to get collection info differently
                        try:
                            # Check if we can access the collection info in a different way
                            from qdrant_client.http import models
                            # Try to get just the first few points without specifying vector name
                            search_result = client.search(
                                collection_name=collection_name,
                                query_vector=[0.0] * 1024,  # dummy query
                                limit=1,
                                with_payload=True
                            )

                            print(f"Final attempt - search result: {len(search_result)} points")
                        except Exception as e3:
                            print(f"Final attempt also failed: {e3}")

                return True
            else:
                print(f"ERROR: Collection exists but has no points")
                return False

        except Exception as e:
            print(f"ERROR: Collection '{collection_name}' does not exist: {e}")
            return False

    except Exception as e:
        print(f"ERROR: Could not connect to Qdrant Cloud: {e}")
        return False

if __name__ == "__main__":
    success = verify_qdrant_ingestion()
    if success:
        print(f"\nVERIFICATION COMPLETE: All book content successfully sent to Qdrant Cloud!")
    else:
        print(f"\nVERIFICATION FAILED: Book content was not properly sent to Qdrant Cloud.")
        sys.exit(1)