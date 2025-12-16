#!/usr/bin/env python3
"""
Test verification script to confirm the chatbot improvements work as expected.
This script verifies that the RAG model provides consistent responses across both modes.
"""

import asyncio
import sys
import os

# Add the backend directory to the path so we can import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.models.rag import RAGModel, RAGResponse


async def test_chatbot_improvements():
    """Test that the chatbot improvements work correctly"""
    print("Testing chatbot improvements after implementation...")
    print("="*60)

    # Initialize the RAG model
    print("Initializing RAG model...")
    try:
        rag_model = RAGModel()
        print("PASS: RAG model initialized successfully")
    except Exception as e:
        print(f"FAIL: Failed to initialize RAG model: {e}")
        return False

    # Test query that was problematic before
    query = "what is NVIDIA Isaac Platform ?"
    print(f"\nTesting query: '{query}'")

    # Test 1: Full-book mode
    print("\nTesting full-book mode...")
    try:
        response_full = await rag_model.process_query(query, mode="full_book")
        print(f"   Response preview: {response_full.response[:100]}...")
        print(f"   Sources retrieved: {len(response_full.sources)}")
        print(f"   References: {len(response_full.references)}")

        if "The book does not provide details about this topic." in response_full.response:
            print("   FAIL: Full-book mode still returns fallback message")
            full_book_success = False
        else:
            print("   PASS: Full-book mode returns meaningful content")
            full_book_success = True
    except Exception as e:
        print(f"   FAIL: Error in full-book mode: {e}")
        full_book_success = False

    # Test 2: Selected-text mode with relevant text
    print("\nTesting selected-text mode with relevant text...")
    relevant_text = "NVIDIA Isaac Platform is a comprehensive, end-to-end robotics development solution built on NVIDIA's Omniverse platform."
    try:
        response_selected = await rag_model.process_query(query, mode="selected_text", selected_text=relevant_text)
        print(f"   Response preview: {response_selected.response[:100]}...")
        print(f"   Sources retrieved: {len(response_selected.sources)}")
        print(f"   References: {len(response_selected.references)}")

        if "The book does not provide details about this topic." in response_selected.response:
            print("   FAIL: Selected-text mode returns fallback message")
            selected_text_success = False
        else:
            print("   PASS: Selected-text mode returns meaningful content")
            selected_text_success = True
    except Exception as e:
        print(f"   FAIL: Error in selected-text mode: {e}")
        selected_text_success = False

    # Test 3: Selected-text mode with irrelevant text (should still work due to our fix)
    print("\nTesting selected-text mode with irrelevant text (fallback test)...")
    irrelevant_text = "This is some completely irrelevant text that has nothing to do with NVIDIA Isaac Platform."
    try:
        response_irrelevant = await rag_model.process_query(query, mode="selected_text", selected_text=irrelevant_text)
        print(f"   Response preview: {response_irrelevant.response[:100]}...")
        print(f"   Sources retrieved: {len(response_irrelevant.sources)}")
        print(f"   References: {len(response_irrelevant.references)}")

        if "The book does not provide details about this topic." in response_irrelevant.response:
            print("   FAIL: Selected-text with irrelevant text returns fallback (fix may not be working)")
            fallback_success = False
        else:
            print("   PASS: Selected-text with irrelevant text still finds relevant info (fix working!)")
            fallback_success = True
    except Exception as e:
        print(f"   FAIL: Error in selected-text mode with irrelevant text: {e}")
        fallback_success = False

    # Test 4: Consistency check - both modes should provide similar quality responses
    print("\nTesting response consistency between modes...")
    try:
        # Use a common query
        consistency_query = "what is humanoid robotics?"
        response_full_consistency = await rag_model.process_query(consistency_query, mode="full_book")
        response_selected_consistency = await rag_model.process_query(
            consistency_query,
            mode="selected_text",
            selected_text="This is a text about humanoid robotics."
        )

        # Check if both responses avoid the fallback message
        full_has_content = "The book does not provide details about this topic." not in response_full_consistency.response
        selected_has_content = "The book does not provide details about this topic." not in response_selected_consistency.response

        if full_has_content and selected_has_content:
            print("   PASS: Both modes provide meaningful responses (consistency achieved)")
            consistency_success = True
        else:
            print("   FAIL: Inconsistent responses between modes")
            consistency_success = False

    except Exception as e:
        print(f"   FAIL: Error in consistency test: {e}")
        consistency_success = False

    # Summary
    print("\n" + "="*60)
    print("TEST RESULTS SUMMARY:")
    print(f"   Full-book mode: {'PASS' if full_book_success else 'FAIL'}")
    print(f"   Selected-text mode: {'PASS' if selected_text_success else 'FAIL'}")
    print(f"   Fallback mechanism: {'PASS' if fallback_success else 'FAIL'}")
    print(f"   Response consistency: {'PASS' if consistency_success else 'FAIL'}")

    all_tests_passed = all([full_book_success, selected_text_success, fallback_success, consistency_success])

    print(f"\nOverall result: {'ALL TESTS PASSED' if all_tests_passed else 'SOME TESTS FAILED'}")
    print("="*60)

    return all_tests_passed


async def main():
    """Main function to run the verification tests"""
    print("Starting chatbot improvement verification...")
    success = await test_chatbot_improvements()

    if success:
        print("\nAll verification tests passed! The chatbot improvements are working correctly.")
        print("\nImprovements verified:")
        print("   • Consistent responses across both query modes")
        print("   • Selected-text mode falls back to full database when needed")
        print("   • Meaningful responses instead of fallback messages for valid queries")
        print("   • Response quality maintained across different scenarios")
    else:
        print("\nSome verification tests failed. Please review the implementation.")

    return success


if __name__ == "__main__":
    asyncio.run(main())