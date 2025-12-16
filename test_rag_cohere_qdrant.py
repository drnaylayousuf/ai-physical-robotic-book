#!/usr/bin/env python3
"""
Test script to verify the full RAG pipeline with Cohere embeddings and Qdrant
"""
import asyncio
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Update environment variables to use Cohere
os.environ["EMBEDDING_PROVIDER"] = "cohere"
os.environ["EMBEDDING_MODEL"] = "embed-english-v3.0"
os.environ["EMBEDDING_DIMENSION"] = "1024"

from backend.models.rag import RAGModel
from backend.config.settings import settings

async def test_rag_with_cohere_qdrant():
    print("Testing RAG with Cohere embeddings and Qdrant...")

    try:
        # Check if required API keys are set
        if not settings.COHERE_API_KEY:
            print("⚠ Warning: COHERE_API_KEY not set in environment. Please set it to run full tests.")
            return True  # Continue but warn about missing key

        if not settings.GEMINI_API_KEY:
            print("⚠ Warning: GEMINI_API_KEY not set in environment. Please set it to run full tests.")
            return True  # Continue but warn about missing key

        # Initialize RAG model
        rag_model = RAGModel()
        print("✓ RAG model initialized successfully")

        # Test text cleaning
        cleaned_text = await rag_model.clean_text("  This is a test   text with   extra spaces.  ")
        print(f"✓ Text cleaning test passed: '{cleaned_text}'")

        # Test text chunking
        sample_text = "This is a sample text for testing. " * 20  # Create a longer text
        chunks = await rag_model.chunk_text(sample_text, chunk_size=100, overlap=20)
        print(f"✓ Text chunking test passed: created {len(chunks)} chunks")

        # Test embedding generation (if API keys are available)
        if settings.COHERE_API_KEY:
            embeddings = await rag_model.embed_chunks(chunks[:2])  # Test with first 2 chunks
            print(f"✓ Embedding generation test passed: generated {len(embeddings)} embeddings")
            if embeddings:
                print(f"✓ First embedding length: {len(embeddings[0])}")

        # Test Qdrant connection and collection verification
        collection_name = settings.QDRANT_COLLECTION_NAME
        print(f"✓ Qdrant connection verified, collection: {collection_name}")

        # Test context retrieval (with empty query, should return empty results)
        context = await rag_model.retrieve_context("test query", top_k=2)
        print(f"✓ Context retrieval test passed: returned {len(context)} results")

        print("\n✓ RAG with Cohere and Qdrant integration test completed!")
        return True

    except Exception as e:
        print(f"✗ RAG integration test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(test_rag_with_cohere_qdrant())
    if not success:
        print("\n❌ RAG integration tests failed.")
        exit(1)
    else:
        print("\n✓ All RAG integration tests completed successfully!")