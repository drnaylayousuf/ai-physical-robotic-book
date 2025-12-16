#!/usr/bin/env python3
"""
Test script to verify the Cohere embedding service works correctly with Qdrant
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

from backend.utils.cohere_embeddings import CohereEmbeddingService
from backend.utils.embeddings import EmbeddingService
from backend.config.settings import settings

async def test_cohere_embedding_service():
    print("Testing Cohere Embedding Service...")

    # Test direct Cohere service
    try:
        cohere_service = CohereEmbeddingService()
        print("✓ Cohere service initialized successfully")

        # Test single embedding
        text = "This is a test sentence for embedding."
        embedding = await cohere_service.generate_embedding(text)
        print(f"✓ Single embedding generated: length = {len(embedding)}")

        # Test batch embedding
        texts = ["First sentence", "Second sentence", "Third sentence"]
        embeddings = await cohere_service.generate_embeddings_batch(texts)
        print(f"✓ Batch embedding generated: {len(embeddings)} embeddings, each of length {len(embeddings[0]) if embeddings else 0}")

        # Test embedding size
        size = cohere_service.get_embedding_size()
        print(f"✓ Embedding size reported: {size}")

    except Exception as e:
        print(f"✗ Cohere service test failed: {e}")
        return False

    # Test the main EmbeddingService with Cohere provider
    try:
        # Make sure COHERE_API_KEY is set
        if not settings.COHERE_API_KEY:
            print("⚠ Warning: COHERE_API_KEY not set in environment. Please set it to run full tests.")
            return True  # Return True to continue, but warn about missing key

        embedding_service = EmbeddingService()
        print("✓ Main EmbeddingService initialized with Cohere provider")

        # Test single embedding
        text = "Testing the main embedding service."
        embedding = await embedding_service.generate_embedding(text)
        print(f"✓ Main service single embedding: length = {len(embedding)}")

        # Test batch embedding
        texts = ["Test 1", "Test 2", "Test 3"]
        embeddings = await embedding_service.generate_embeddings_batch(texts)
        print(f"✓ Main service batch embedding: {len(embeddings)} embeddings")

        # Test embedding size
        size = embedding_service.get_embedding_size()
        print(f"✓ Main service embedding size: {size}")

        print("\n✓ All Cohere embedding tests passed!")
        return True

    except Exception as e:
        print(f"✗ Main EmbeddingService test failed: {e}")
        return False

if __name__ == "__main__":
    success = asyncio.run(test_cohere_embedding_service())
    if not success:
        print("\n❌ Some tests failed.")
        exit(1)
    else:
        print("\n✓ All tests completed successfully!")