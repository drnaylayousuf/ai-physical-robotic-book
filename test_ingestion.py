#!/usr/bin/env python3
"""
Script to test content ingestion into Qdrant using the RAG model
"""
import asyncio
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.models.rag import RAGModel
from backend.config.settings import settings

async def test_content_ingestion():
    """Test content ingestion into Qdrant"""
    print("Testing Content Ingestion to Qdrant Cloud")
    print("=" * 50)
    print(f"Qdrant URL: {settings.QDRANT_URL}")
    print(f"Collection: {settings.QDRANT_COLLECTION_NAME}")
    print(f"Embedding Provider: {settings.EMBEDDING_PROVIDER}")
    print(f"Embedding Model: {settings.EMBEDDING_MODEL}")
    print(f"Embedding Dimensions: {settings.EMBEDDING_DIMENSION}")
    print()

    try:
        # Initialize RAG model
        print("1. Initializing RAG model...")
        rag_model = RAGModel()
        print("   RAG model initialized successfully!")
        print()

        # Read test content
        print("2. Reading test content...")
        with open('test_content.txt', 'r', encoding='utf-8') as f:
            content = f.read()
        print(f"   Content length: {len(content)} characters")
        print()

        # Ingest content into Qdrant
        print("3. Ingesting content into Qdrant...")
        print("   This will clean, chunk, embed, and store the content...")

        result = await rag_model.ingest_content(content)

        print(f"   Chunks processed: {result['chunks_processed']}")
        print(f"   Collection name: {result['collection_name']}")
        print(f"   Chunk IDs: {result['chunk_ids'][:3]}...")  # Show first 3 IDs
        print("   Content ingestion completed successfully!")
        print()

        # Test retrieval after ingestion
        print("4. Testing retrieval from ingested content...")
        query_results = await rag_model.retrieve_context("What are the main components of humanoid robots?", top_k=3)
        print(f"   Retrieved {len(query_results)} results for query")

        if query_results:
            print("   Sample results:")
            for i, result in enumerate(query_results[:2]):  # Show first 2 results
                print(f"     Result {i+1}:")
                print(f"       Content preview: {result['content'][:100]}...")
                print(f"       Score: {result['score']:.3f}")
                print()

        print("5. Content ingestion and retrieval test completed successfully!")
        print()
        print("SUCCESS: Your Qdrant Cloud is now ready to store and retrieve your humanoid robotics book content!")
        print("The chatbot will be able to answer questions based on your ingested content.")

        return True

    except Exception as e:
        print(f"ERROR during content ingestion: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(test_content_ingestion())

    if success:
        print("\nCONTENT INGESTION: SUCCESSFUL!")
    else:
        print("\nCONTENT INGESTION: FAILED")

    sys.exit(0 if success else 1)