#!/usr/bin/env python3
"""
Simple test to verify that the /api/ask endpoint works without authentication
"""
import requests
import json

def test_chat_endpoint():
    """Test the chat endpoint without authentication"""
    print("Testing /api/ask endpoint...")

    try:
        # Test the endpoint without any authentication
        response = requests.post(
            "http://localhost:8000/ask",  # Use path as shown in OpenAPI spec
            headers={"Content-Type": "application/json"},
            data=json.dumps({
                "question": "What is humanoid robotics?",
                "mode": "full_book",
                "selected_text": None
            }),
            timeout=30  # Give it more time for RAG processing
        )

        print(f"Response status: {response.status_code}")

        if response.status_code == 200:
            print("SUCCESS: Chat endpoint is accessible without authentication!")
            try:
                data = response.json()
                print(f"Response: {data.get('response', 'No response text')[:100]}...")
            except:
                print(f"Raw response: {response.text[:200]}")
        elif response.status_code == 422:
            print("VALIDATION ERROR: The endpoint exists but has validation requirements")
            print(f"Response: {response.text}")
        elif response.status_code in [401, 403]:
            print("AUTHENTICATION REQUIRED: The endpoint still requires authentication")
            print(f"Response: {response.text}")
        else:
            print(f"OTHER ERROR: Status code {response.status_code}")
            print(f"Response: {response.text}")

    except requests.exceptions.ConnectionError:
        print("ERROR: Cannot connect to the server. Is it running on http://localhost:8000?")
    except Exception as e:
        print(f"ERROR: {e}")

if __name__ == "__main__":
    test_chat_endpoint()