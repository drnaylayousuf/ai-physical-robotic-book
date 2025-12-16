#!/usr/bin/env python3
"""
Script to test full RAG query functionality with your Qdrant Cloud
"""
import asyncio
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.models.rag import RAGModel
from backend.config.settings import settings

async def test_full_rag_query():
    """Test full RAG query functionality"""
    print("Testing Full RAG Query with Qdrant Cloud")
    print("=" * 50)
    print(f"Qdrant URL: {settings.QDRANT_URL}")
    print(f"Collection: {settings.QDRANT_COLLECTION_NAME}")
    print(f"Embedding Provider: {settings.EMBEDDING_PROVIDER}")
    print()

    try:
        # Initialize RAG model
        print("1. Initializing RAG model...")
        rag_model = RAGModel()
        print("   RAG model initialized successfully!")
        print()

        # Test query that should match the content we ingested
        print("2. Testing query against ingested content...")
        query = "What are the main components of humanoid robots?"

        print(f"   Query: {query}")
        result = await rag_model.process_query(query, mode="full_book", top_k=2)

        print(f"   Response: {result.response[:200]}...")
        print(f"   Sources: {len(result.sources)} retrieved")
        print(f"   References: {result.references}")
        print()

        # Show detailed source information
        if result.sources:
            print("   Detailed Sources:")
            for i, source in enumerate(result.sources[:2]):  # Show first 2 sources
                print(f"     Source {i+1}:")
                print(f"       Content: {source['content'][:100]}...")
                print(f"       Score: {source['score']:.3f}")
                print(f"       Source: {source['source']}")
                print()

        print("3. Full RAG query test completed successfully!")
        print()
        print("SUCCESS: Your Qdrant Cloud integration is fully functional!")
        print("You can now ask questions about your humanoid robotics content.")

        return True

    except Exception as e:
        print(f"ERROR during full RAG query: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(test_full_rag_query())

    if success:
        print("\nFULL RAG QUERY: SUCCESSFUL!")
    else:
        print("\nFULL RAG QUERY: FAILED")

    sys.exit(0 if success else 1)