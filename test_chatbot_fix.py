#!/usr/bin/env python3
"""
Test script to verify the chatbot fix for consistent responses.
This script tests the RAG model directly to ensure that both selected_text and full_book modes
provide consistent, meaningful responses.
"""

import asyncio
import sys
import os

# Add the backend directory to the path so we can import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.models.rag import RAGModel, RAGResponse


async def test_chatbot_consistency():
    """Test that the chatbot provides consistent responses regardless of mode"""
    print("Testing chatbot consistency after fixes...")

    # Initialize the RAG model
    rag_model = RAGModel()

    # Test query that was problematic before
    query = "what is NVIDIA Isaac Platform ?"

    print(f"\n1. Testing query: '{query}'")

    # Test full_book mode
    print("\n2. Testing full_book mode...")
    try:
        response_full = await rag_model.process_query(query, mode="full_book")
        print(f"   Response: {response_full.response[:100]}...")
        print(f"   Sources: {len(response_full.sources)} chunks retrieved")
        print(f"   References: {len(response_full.references)} references")
    except Exception as e:
        print(f"   Error in full_book mode: {e}")
        response_full = None

    # Test selected_text mode with a relevant text snippet
    print("\n3. Testing selected_text mode with relevant text...")
    selected_text = "NVIDIA Isaac Platform is a comprehensive, end-to-end robotics development solution built on NVIDIA's Omniverse platform. It is a unified framework consisting of a suite of tools and SDKs designed to meet the demands for powerful simulation, accelerated AI development, and seamless deployment on specialized hardware."

    try:
        response_selected = await rag_model.process_query(query, mode="selected_text", selected_text=selected_text)
        print(f"   Response: {response_selected.response[:100]}...")
        print(f"   Sources: {len(response_selected.sources)} chunks retrieved")
        print(f"   References: {len(response_selected.references)} references")
    except Exception as e:
        print(f"   Error in selected_text mode: {e}")
        response_selected = None

    # Test selected_text mode with irrelevant text (should still work due to our fix)
    print("\n4. Testing selected_text mode with irrelevant text (should still work due to our fix)...")
    irrelevant_text = "This is some completely irrelevant text that has nothing to do with NVIDIA Isaac Platform."

    try:
        response_irrelevant = await rag_model.process_query(query, mode="selected_text", selected_text=irrelevant_text)
        print(f"   Response: {response_irrelevant.response[:100]}...")
        print(f"   Sources: {len(response_irrelevant.sources)} chunks retrieved")
        print(f"   References: {len(response_irrelevant.references)} references")
    except Exception as e:
        print(f"   Error in selected_text mode with irrelevant text: {e}")
        response_irrelevant = None

    # Compare responses
    print("\n5. Analysis:")
    if response_full and response_selected:
        if "The book does not provide details about this topic." in response_full.response:
            print("   [X] Full-book mode still returns no details")
        else:
            print("   [V] Full-book mode returns meaningful content")

        if "The book does not provide details about this topic." in response_selected.response:
            print("   [X] Selected-text mode still returns no details")
        else:
            print("   [V] Selected-text mode returns meaningful content")

    if response_irrelevant:
        # With our fix, even irrelevant selected text should find relevant info from the full database
        if "The book does not provide details about this topic." in response_irrelevant.response:
            print("   [X] Selected-text with irrelevant text still returns no details (fix may not be working)")
        else:
            print("   [V] Selected-text with irrelevant text still finds relevant info from full database (fix working!)")

    print("\n6. Summary:")
    print("   The fix ensures that:")
    print("   - selected_text mode now searches the full database as backup")
    print("   - both modes should provide consistent, meaningful responses")
    print("   - irrelevant selected text doesn't break the response quality")


if __name__ == "__main__":
    asyncio.run(test_chatbot_consistency())