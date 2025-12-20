#!/usr/bin/env python3
"""
Test script to verify the embedding service is working properly
"""

import os
import sys
import asyncio
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.utils.embeddings import EmbeddingService
from backend.config.settings import settings

async def test_embeddings():
    """Test the embedding service to verify it's generating proper vectors"""
    print("Testing embedding service...")

    try:
        # Initialize the embedding service
        embedding_service = EmbeddingService()
        print(f"Embedding provider: {settings.EMBEDDING_PROVIDER}")
        print(f"Embedding model: {settings.EMBEDDING_MODEL}")
        print(f"Embedding dimension: {settings.EMBEDDING_DIMENSION}")

        # Test text for embedding
        test_text = "humanoid robotics"

        print(f"\nGenerating embedding for: '{test_text}'")
        embedding = await embedding_service.generate_embedding(test_text)

        print(f"Embedding length: {len(embedding)}")
        print(f"First 10 values: {embedding[:10]}")

        # Check if embedding is all zeros (which would indicate a problem)
        if all(v == 0.0 for v in embedding):
            print("ERROR: Embedding is all zeros - service not working properly!")
            return False
        else:
            print("SUCCESS: Embedding service is working properly!")

            # Check for reasonable values (not all the same)
            unique_values = set(embedding)
            if len(unique_values) < 10:  # If too few unique values, might be problematic
                print("WARNING: Embedding has very few unique values - may indicate poor quality")
            else:
                print("SUCCESS: Embedding has good variety of values")

        # Test another text
        test_text2 = "API reference"
        print(f"\nGenerating embedding for: '{test_text2}'")
        embedding2 = await embedding_service.generate_embedding(test_text2)

        # Check similarity between embeddings (should be different for different content)
        similarity = cosine_similarity(embedding, embedding2)
        print(f"Cosine similarity between '{test_text}' and '{test_text2}': {similarity}")

        if abs(similarity) > 0.9:  # If too similar, might indicate a problem
            print("WARNING: Embeddings are too similar - may indicate poor differentiation")
        else:
            print("SUCCESS: Embeddings are appropriately different for different content")

        return True

    except Exception as e:
        print(f"ERROR: Failed to test embedding service: {e}")
        return False

def cosine_similarity(vec1, vec2):
    """Calculate cosine similarity between two vectors"""
    dot_product = sum(a * b for a, b in zip(vec1, vec2))
    magnitude1 = sum(a * a for a in vec1) ** 0.5
    magnitude2 = sum(b * b for b in vec2) ** 0.5

    if magnitude1 == 0 or magnitude2 == 0:
        return 0.0

    return dot_product / (magnitude1 * magnitude2)

if __name__ == "__main__":
    success = asyncio.run(test_embeddings())
    if success:
        print("\nSUCCESS: Embedding service test completed successfully!")
    else:
        print("\nERROR: Embedding service test failed!")
        sys.exit(1)