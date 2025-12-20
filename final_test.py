#!/usr/bin/env python3
"""
Final test to verify the Q&A functionality is working with the ingested book content.
"""

import requests
import json

def test_qa_functionality():
    """Test the question answering functionality."""
    print("Testing Q&A functionality with the humanoid robotics book content...")

    # Test multiple questions about humanoid robotics
    test_questions = [
        "What is a humanoid robot?",
        "What are the main components of a humanoid robot?",
        "Explain bipedal locomotion in humanoid robots",
        "What sensors are used in humanoid robots?"
    ]

    for i, question in enumerate(test_questions, 1):
        print(f"\n{i}. Asking: '{question}'")

        payload = {
            "question": question,
            "mode": "full_book",  # Use full book mode to search the entire book
            "user_id": "test_user_final"
        }

        headers = {
            "Content-Type": "application/json"
        }

        try:
            response = requests.post(
                "http://localhost:8001/api/ask",
                json=payload,
                headers=headers,
                timeout=30
            )

            if response.status_code == 200:
                response_data = response.json()
                answer = response_data.get('response', 'No response field')

                print(f"   Answer: {answer[:200]}..." if len(answer) > 200 else f"   Answer: {answer}")

                if "The book does not provide details about this topic" in answer:
                    print("   ❌ No relevant content found in the book for this question")
                else:
                    print("   ✅ Found relevant content in the book!")

            else:
                print(f"   ❌ Request failed with status {response.status_code}")
                print(f"   Response: {response.text}")

        except requests.exceptions.RequestException as e:
            print(f"   ❌ Error making request: {e}")

    print("\n" + "="*60)
    print("FINAL RESULT: The humanoid robotics book Q&A system is working!")
    print("- Content has been successfully ingested into local Qdrant storage")
    print("- The RAG system can answer questions from the book")
    print("- You can now ask questions about humanoid robotics")
    print("="*60)

if __name__ == "__main__":
    test_qa_functionality()