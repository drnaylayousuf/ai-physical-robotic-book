#!/usr/bin/env python3
"""
Test script to verify that the QA functionality is working with the ingested content.
"""

import requests
import json

def test_qa_functionality():
    """Test the question answering functionality."""
    print("Testing question answering functionality...")

    # Example question about humanoid robotics
    payload = {
        "question": "What is a humanoid robot?",
        "mode": "full_book",  # Use full book mode to search the entire book
        "user_id": "test_user_123"
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

        print(f"QA response status: {response.status_code}")
        if response.status_code == 200:
            response_data = response.json()
            print(f"Response: {json.dumps(response_data, indent=2)}")

            answer = response_data.get('response', 'No response field')
            sources = response_data.get('sources', [])
            references = response_data.get('references', [])

            print(f"\nAnswer: {answer[:500]}...")  # First 500 chars
            print(f"Number of sources: {len(sources)}")
            print(f"References: {references}")

            if "The book does not provide details about this topic" in answer:
                print("\n⚠️  The system couldn't find relevant information in the ingested content.")
                print("   This could be because:")
                print("   - The content is still being processed")
                print("   - The specific topic isn't covered in the book")
                print("   - The embeddings are still being generated")
            else:
                print("\n✅ SUCCESS: The system responded with information from the book!")

            return True
        else:
            print(f"QA failed with status {response.status_code}")
            print(f"Response: {response.text}")
            return False
    except Exception as e:
        print(f"Error testing QA: {e}")
        return False

if __name__ == "__main__":
    test_qa_functionality()