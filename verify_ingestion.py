#!/usr/bin/env python3
"""
Script to verify that book content has been properly ingested into Qdrant.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.config.settings import settings
from qdrant_client import QdrantClient
from qdrant_client.http import models

def verify_qdrant_ingestion():
    """Verify that the book content has been ingested into Qdrant."""
    try:
        # Initialize Qdrant client
        if settings.QDRANT_API_KEY:
            qdrant_client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                timeout=10
            )
        else:
            qdrant_client = QdrantClient(
                url=settings.QDRANT_URL,
                timeout=10
            )

        # Get collection info
        collection_name = settings.QDRANT_COLLECTION_NAME
        collection_info = qdrant_client.get_collection(collection_name)

        print(f"Collection '{collection_name}' exists")
        print(f"Points count: {collection_info.points_count}")
        print(f"Vector size: {collection_info.config.params.vectors.size if hasattr(collection_info.config.params, 'vectors') else collection_info.config.params.size}")

        # Check a few sample points to verify content
        if collection_info.points_count > 0:
            # Get a few sample points using scroll
            scroll_result = qdrant_client.scroll(
                collection_name=collection_name,
                limit=3,
                with_payload=True,
                with_vectors=False
            )

            # The scroll method returns (records, next_page_offset)
            records, _ = scroll_result
            print(f"\nSample points from collection:")
            for i, record in enumerate(records):
                content_preview = record.payload.get('content', '')[:200] + "..." if len(record.payload.get('content', '')) > 200 else record.payload.get('content', '')
                print(f"Point {i+1} (ID: {record.id}):")
                print(f"  Content preview: {content_preview}")
                print()

        return collection_info.points_count

    except Exception as e:
        print(f"Error verifying Qdrant ingestion: {e}")
        return 0

if __name__ == "__main__":
    print("Verifying Qdrant ingestion...")
    points_count = verify_qdrant_ingestion()

    if points_count > 0:
        print(f"Verification successful! {points_count} points found in Qdrant collection.")
    else:
        print("Verification failed! No points found in Qdrant collection.")