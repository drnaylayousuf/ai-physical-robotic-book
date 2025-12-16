#!/usr/bin/env python3
"""
Verification script to confirm Qdrant connection is fixed
"""
import asyncio
import sys
import os

# Add backend to path to use the same configuration as your app
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.models.rag import RAGModel
from backend.config.settings import settings

async def verify_fix():
    """Verify that the Qdrant connection issue is fixed"""
    print("Verifying Qdrant connection fix...")
    print(f"Qdrant URL: {settings.QDRANT_URL}")
    print(f"Collection: {settings.QDRANT_COLLECTION_NAME}")
    print()

    try:
        # Test 1: Initialize RAG model (tests connection)
        print("1. Testing RAG model initialization...")
        rag_model = RAGModel()
        print("   RAG model initialized successfully - Qdrant connection established!")
        print()

        # Test 2: Test text processing functions
        print("2. Testing text processing functions...")
        test_text = "This is a test document about humanoid robotics and AI."
        cleaned = await rag_model.clean_text(test_text)
        chunks = await rag_model.chunk_text(cleaned, chunk_size=100, overlap=20)
        print(f"   Text processing works - created {len(chunks)} chunks")
        print()

        # Test 3: Test embedding generation (will use fallback)
        print("3. Testing embedding generation...")
        try:
            embeddings = await rag_model.embed_chunks(chunks[:1])  # Just first chunk
            print(f"   Embedding generation works - created {len(embeddings[0])} dimensional embedding")
        except Exception as e:
            print(f"   Embedding generation issue (expected if API keys not configured): {e}")
        print()

        # Test 4: Test storing data in Qdrant
        print("4. Testing Qdrant storage...")
        collection_name = f"test_{settings.QDRANT_COLLECTION_NAME}"
        test_embeddings = [[0.1] * 384]  # Use dummy embedding for testing
        chunk_ids = await rag_model.store_chunks(chunks, test_embeddings, collection_name)
        print(f"   Successfully stored {len(chunk_ids)} chunk(s) in Qdrant collection '{collection_name}'")
        print()

        # Test 5: Test retrieving data from Qdrant
        print("5. Testing Qdrant retrieval...")
        results = await rag_model.retrieve_context("humanoid robotics", top_k=1, collection_name=collection_name)
        print(f"   Successfully retrieved {len(results)} result(s) from Qdrant")
        print()

        # Clean up test data
        from qdrant_client import QdrantClient
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

        try:
            cleanup_client.delete_collection(collection_name)
            print(f"   Cleaned up test collection: {collection_name}")
        except:
            pass  # Ignore cleanup errors

        print()
        print("VERIFICATION COMPLETE!")
        print("QDRANT CONNECTION IS WORKING PROPERLY!")
        print("The 'Failed to fetch' and 'no connection in qdrant' errors should be fixed!")
        print()
        print("Next steps:")
        print("   1. Start your backend service: python -m backend.main")
        print("   2. Ask questions in the chatbot about the book")
        print("   3. The chatbot will now properly query Qdrant for book content")

        return True

    except Exception as e:
        print(f"VERIFICATION FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("Qdrant Connection Fix Verification")
    print("=" * 50)

    success = asyncio.run(verify_fix())

    print("=" * 50)
    if success:
        print("FIX VERIFICATION: PASSED")
        print("Your Qdrant connection issue has been resolved!")
    else:
        print("FIX VERIFICATION: FAILED")
        print("Please check the error messages above.")

    sys.exit(0 if success else 1)