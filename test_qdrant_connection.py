#!/usr/bin/env python3
"""
Test script to verify Qdrant connection and RAG model integration
"""
import asyncio
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.models.rag import RAGModel
from backend.config.settings import settings

async def test_qdrant_connection():
    """Test Qdrant connection and basic functionality"""
    print("Testing Qdrant connection...")

    try:
        # Initialize RAG model - this will attempt to connect to Qdrant
        rag_model = RAGModel()
        print(f"SUCCESS: Successfully connected to Qdrant at {settings.QDRANT_URL}")

        # Test basic functionality - clean and chunk some text
        test_text = "This is a test document about humanoid robotics. It contains information about AI and physical systems."
        cleaned_text = await rag_model.clean_text(test_text)
        print(f"SUCCESS: Text cleaning works: '{cleaned_text[:50]}...'")

        chunks = await rag_model.chunk_text(cleaned_text, chunk_size=100, overlap=20)
        print(f"SUCCESS: Text chunking works: {len(chunks)} chunks created")

        # Test embedding generation
        embeddings = await rag_model.embed_chunks(chunks[:1])  # Only first chunk to save time
        print(f"SUCCESS: Embedding generation works: {len(embeddings[0])} dimensions")

        # Test storing chunks (this will create a collection if it doesn't exist)
        collection_name = "test_collection"
        chunk_ids = await rag_model.store_chunks(chunks[:1], embeddings, collection_name)
        print(f"SUCCESS: Chunk storage works: {len(chunk_ids)} chunks stored in '{collection_name}'")

        # Test retrieval
        query = "What is this document about?"
        results = await rag_model.retrieve_context(query, top_k=1, collection_name=collection_name)
        print(f"SUCCESS: Context retrieval works: {len(results)} results retrieved")

        if results:
            print(f"  - First result: '{results[0]['content'][:50]}...'")
            print(f"  - Score: {results[0]['score']:.3f}")

        print("\nSUCCESS: All tests passed! Qdrant integration is working correctly.")
        return True

    except ConnectionError as e:
        print(f"ERROR: Connection error: {e}")
        print("Make sure Qdrant service is running at the configured URL.")
        print("If running locally, start Qdrant with: docker run -p 6333:6333 qdrant/qdrant")
        return False
    except Exception as e:
        print(f"ERROR: Error during testing: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(test_qdrant_connection())
    sys.exit(0 if success else 1)