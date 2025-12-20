#!/usr/bin/env python3
"""
Test script to verify that your local Qdrant setup is working with the humanoid robotics book project.
This script sends a question to your running application and checks the response.
"""

import requests
import json
import time

def test_book_qa():
    """Test the book question answering functionality."""

    # Wait a moment to ensure the server is fully ready
    time.sleep(2)

    # First, let's try to generate an admin token (as seen in your ingest_book_content.py)
    print("Attempting to generate admin token...")

    # Try to call the ask endpoint
    url = "http://localhost:8000/api/ask"

    # Example question about humanoid robotics
    payload = {
        "question": "What is a humanoid robot?",
        "mode": "full_book",  # Use full book mode to search the entire book
        "user_id": "test_user_123"
    }

    headers = {
        "Content-Type": "application/json"
    }

    print(f"Sending request to: {url}")
    print(f"Payload: {json.dumps(payload, indent=2)}")

    try:
        response = requests.post(url, json=payload, headers=headers, timeout=30)
        print(f"Response Status Code: {response.status_code}")
        print(f"Response Headers: {dict(response.headers)}")

        if response.status_code == 200:
            response_data = response.json()
            print(f"Response Data: {json.dumps(response_data, indent=2)}")

            if "response" in response_data:
                print("\n" + "="*50)
                print("SUCCESS! Your application responded to the question:")
                print(f"Question: {payload['question']}")
                print(f"Answer: {response_data['response']}")
                print("="*50)
            else:
                print("Response doesn't contain expected 'response' field")
                print(f"Full response: {response_data}")
        else:
            print(f"Request failed with status code: {response.status_code}")
            print(f"Response text: {response.text}")

            # Try to get more information by checking if the server is running
            try:
                health_check = requests.get("http://localhost:8000/health", timeout=10)
                print(f"Health check status: {health_check.status_code}")
                if health_check.status_code == 200:
                    print("Server is running but the specific endpoint might have issues")
            except requests.exceptions.RequestException:
                print("Server might not be responding properly")

    except requests.exceptions.ConnectionError:
        print("ERROR: Cannot connect to the server. Is it running on http://localhost:8000?")
        print("Make sure you've started the backend with: uvicorn main:app --reload")
    except requests.exceptions.Timeout:
        print("ERROR: Request timed out. The server might be slow to respond or not running properly.")
    except requests.exceptions.RequestException as e:
        print(f"ERROR: Request failed with exception: {e}")
    except json.JSONDecodeError:
        print(f"Response text (not JSON): {response.text}")

if __name__ == "__main__":
    test_book_qa()