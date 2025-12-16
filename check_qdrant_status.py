#!/usr/bin/env python3
"""
Quick Qdrant Status Check
This script tests if your Qdrant instance is accessible and working with your current configuration.
"""
import asyncio
import sys
import os

# Add backend to path to use the same configuration as your app
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.models.rag import RAGModel
from backend.config.settings import settings

async def quick_qdrant_check():
    """Quick check to see if Qdrant connection works with your app's configuration"""
    print("Checking Qdrant connection with your application configuration...")
    print(f"URL: {settings.QDRANT_URL}")
    print(f"Collection: {settings.QDRANT_COLLECTION_NAME}")
    print(f"API Key configured: {'Yes' if settings.QDRANT_API_KEY else 'No'}")
    print()

    try:
        # This will test the connection just like your chatbot does
        rag_model = RAGModel()  # This constructor tests the connection
        print("SUCCESS: Qdrant connection established!")
        print()

        # Test a simple operation
        print("Testing basic Qdrant operations...")

        # Create a test collection name for this check
        test_collection = f"{settings.QDRANT_COLLECTION_NAME}_test"

        # Test storing a simple chunk
        test_chunks = ["This is a test chunk about humanoid robotics."]
        test_embeddings = [[0.1] * 384]  # Dummy embedding

        chunk_ids = await rag_model.store_chunks(test_chunks, test_embeddings, test_collection)
        print(f"Successfully stored {len(chunk_ids)} chunk(s) in Qdrant")

        # Test retrieving it back
        results = await rag_model.retrieve_context("test query", top_k=1, collection_name=test_collection)
        print(f"Successfully retrieved {len(results)} result(s) from Qdrant")

        # Clean up test data
        from qdrant_client import QdrantClient
        from qdrant_client.http import models
        import logging

        # Create client to clean up
        if settings.QDRANT_API_KEY:
            cleanup_client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                timeout=10
            )
        else:
            cleanup_client = QdrantClient(
                url=settings.QDRANT_URL,
                timeout=10
            )

        # Delete the test collection
        try:
            cleanup_client.delete_collection(test_collection)
            print(f"Cleaned up test collection: {test_collection}")
        except:
            pass  # Ignore cleanup errors

        print()
        print("QDRANT CONNECTION TEST PASSED!")
        print("Your chatbot should now be able to connect to Qdrant and answer questions.")
        print("Start your backend service and test the chatbot!")

        return True

    except Exception as e:
        print(f"FAILED: {e}")
        print()
        print("Troubleshooting:")
        print("1. Check that your Qdrant service is running")
        print("2. Verify your QDRANT_URL in .env is correct")
        print("3. Ensure your QDRANT_API_KEY is valid (if required)")
        print("4. Check firewall/network settings")
        print("5. Verify the Qdrant service is accessible from your machine")
        return False

if __name__ == "__main__":
    print("Quick Qdrant Status Check")
    print("=" * 40)

    success = asyncio.run(quick_qdrant_check())

    print("=" * 40)
    if success:
        print("QDRANT IS READY FOR YOUR CHATBOT!")
    else:
        print("QDRANT CONNECTION ISSUE DETECTED")

    sys.exit(0 if success else 1)