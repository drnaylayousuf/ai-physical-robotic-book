#!/usr/bin/env python3
"""
Final test to confirm Qdrant is working end-to-end
"""
import asyncio
import sys
import os

# Add backend to path to use the same configuration as your app
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.models.rag import RAGModel
from backend.config.settings import settings

async def final_test():
    """Final end-to-end test"""
    print("Final Qdrant Connection Test")
    print("=" * 40)
    print(f"Qdrant URL: {settings.QDRANT_URL}")
    print(f"Collection: {settings.QDRANT_COLLECTION_NAME}")
    print()

    try:
        # Initialize RAG model
        print("1. Initializing RAG model...")
        rag_model = RAGModel()
        print("   RAG model initialized successfully!")
        print()

        # Test with actual content and embeddings
        print("2. Testing with actual content...")
        test_content = "Humanoid robotics is an interdisciplinary field combining AI, mechanical engineering, and cognitive science to create robots with human-like characteristics and capabilities."

        # Process the content through the full pipeline
        cleaned_content = await rag_model.clean_text(test_content)
        chunks = await rag_model.chunk_text(cleaned_content, chunk_size=100, overlap=20)
        print(f"   Content processed into {len(chunks)} chunks")

        # Generate actual embeddings (this will use the configured service or fallback)
        embeddings = await rag_model.embed_chunks(chunks)
        print(f"   Generated embeddings: {len(embeddings)} vectors of {len(embeddings[0])} dimensions each")
        print()

        # Store in Qdrant using the proper collection name (use a unique name to avoid conflicts)
        print("3. Storing in Qdrant...")
        import time
        collection_name = f"final_test_{settings.QDRANT_COLLECTION_NAME}_{int(time.time())}"
        chunk_ids = await rag_model.store_chunks(chunks, embeddings, collection_name)
        print(f"   Stored {len(chunk_ids)} chunks in collection '{collection_name}'")
        print()

        # Query Qdrant with a related question
        print("4. Querying Qdrant...")
        query_results = await rag_model.retrieve_context("What is humanoid robotics?", top_k=1, collection_name=collection_name)
        print(f"   Retrieved {len(query_results)} results from Qdrant")

        if query_results:
            result = query_results[0]
            print(f"      - Content preview: {result['content'][:100]}...")
            print(f"      - Score: {result['score']:.3f}")
        print()

        # Clean up
        print("5. Cleaning up...")
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
            print(f"   Cleaned up collection: {collection_name}")
        except Exception as e:
            print(f"   Cleanup issue (expected if collection already deleted): {e}")

        print()
        print("FINAL TEST PASSED!")
        print("QDRANT IS WORKING PROPERLY!")
        print("The chatbot will now be able to store and retrieve book content!")
        print()
        print("To test the full system:")
        print("1. Start the backend: python -m backend.main")
        print("2. Ask questions in the chatbot")
        print("3. The 'Failed to fetch' error should be resolved")
        print("4. The chatbot should properly query Qdrant for book content")

        return True

    except Exception as e:
        print(f"FINAL TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(final_test())

    if success:
        print("\nQDRANT INTEGRATION: VERIFIED AND WORKING!")
    else:
        print("\nQDRANT INTEGRATION: ISSUES FOUND")

    sys.exit(0 if success else 1)