#!/usr/bin/env python3
"""
Direct check of what collections exist in the system by creating a RAG model instance.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.models.rag import RAGModel
import asyncio

def check_rag_model_storage():
    """Directly check what storage the RAG model is using."""

    print("Creating RAG model instance to check storage...")

    try:
        # Create a RAG model instance
        rag_model = RAGModel()

        print("RAG Model created successfully")

        # Check if we're using Qdrant client or in-memory storage
        if rag_model.qdrant_client is not None:
            print("✅ Using Qdrant client (either HTTP or local storage)")

            try:
                # Get collections
                collections_info = rag_model.qdrant_client.get_collections()
                collection_names = [c.name for c in collections_info.collections]
                print(f"Available collections: {collection_names}")

                # Check the specific collection
                from backend.config.settings import settings
                collection_name = settings.QDRANT_COLLECTION_NAME
                print(f"Looking for collection: {collection_name}")

                if collection_name in collection_names:
                    collection_info = rag_model.qdrant_client.get_collection(collection_name)
                    print(f"✅ Collection '{collection_name}' exists with {collection_info.points_count} points")

                    # Try to get some points to verify content
                    if collection_info.points_count > 0:
                        # Get first few points
                        scroll_result = rag_model.qdrant_client.scroll(
                            collection_name=collection_name,
                            limit=2,
                            with_payload=True,
                            with_vectors=False
                        )

                        if scroll_result[0]:
                            print("Sample points from collection:")
                            for i, point in enumerate(scroll_result[0][:2]):
                                content_preview = point.payload.get("content", "")[:100] + "..." if len(point.payload.get("content", "")) > 100 else point.payload.get("content", "")
                                print(f"  {i+1}. ID: {point.id}, Content: {content_preview}")
                        else:
                            print("Warning: No points returned from scroll operation")
                    else:
                        print("Info: Collection exists but has 0 points")
                else:
                    print(f"Error: Collection '{collection_name}' does not exist")

            except Exception as e:
                print(f"Error: Error accessing collections: {e}")
                import traceback
                traceback.print_exc()

        else:
            print("Warning: Using in-memory storage instead of Qdrant client")
            print(f"Available in-memory collections: {list(rag_model._collection_storage.keys())}")

            # Check the specific collection in memory
            from backend.config.settings import settings
            collection_name = settings.QDRANT_COLLECTION_NAME
            if collection_name in rag_model._collection_storage:
                collection_size = len(rag_model._collection_storage[collection_name])
                print(f"Success: In-memory collection '{collection_name}' has {collection_size} items")

                # Show sample items
                if collection_size > 0:
                    print("Sample items from in-memory collection:")
                    for i, (chunk_id, chunk_data) in enumerate(list(rag_model._collection_storage[collection_name].items())[:2]):
                        content_preview = chunk_data["content"][:100] + "..." if len(chunk_data["content"]) > 100 else chunk_data["content"]
                        print(f"  {i+1}. ID: {chunk_id}, Content: {content_preview}")
            else:
                print(f"Error: In-memory collection '{collection_name}' does not exist")

    except Exception as e:
        print(f"Error: Error creating RAG model: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    check_rag_model_storage()