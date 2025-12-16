#!/usr/bin/env python3
"""
Script to ask 'What is humanoid robotics?' and show the answer
"""
import asyncio
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.models.rag import RAGModel
from backend.config.settings import settings

async def ask_humanoid_robotics_question():
    """Ask 'What is humanoid robotics?' and show the answer"""
    print("Asking: What is humanoid robotics?")
    print("=" * 60)
    print(f"Connected to Qdrant Cloud: {settings.QDRANT_URL}")
    print(f"Using collection: {settings.QDRANT_COLLECTION_NAME}")
    print()

    try:
        # Initialize RAG model
        rag_model = RAGModel()

        # Ask the question
        query = "What is humanoid robotics?"
        print(f"Query: {query}")
        print()

        # Get the response
        result = await rag_model.process_query(query, mode="full_book", top_k=2)

        print("ANSWER:")
        print(result.response)
        print()

        print("SOURCES USED:")
        print(f"Retrieved {len(result.sources)} relevant chunks:")
        for i, source in enumerate(result.sources):
            print(f"\nSource {i+1} (Score: {source['score']:.3f}):")
            print(f"Content: {source['content'][:200]}...")
            print(f"Source: {source['source']}")

        print(f"\nReferences: {result.references}")

        return True

    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(ask_humanoid_robotics_question())

    if success:
        print("\nQUESTION ANSWERED SUCCESSFULLY!")
    else:
        print("\nFAILED TO ANSWER QUESTION")

    sys.exit(0 if success else 1)