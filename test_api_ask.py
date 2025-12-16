#!/usr/bin/env python3
"""
Test script to verify the /api/ask endpoint is working
"""
import requests
import json

def test_api_ask():
    """Test if the /api/ask endpoint is accessible"""
    print("Testing /api/ask endpoint...")
    print("Checking if backend API is running on http://localhost:8000")
    print()

    try:
        # Test if the endpoint exists (will return 405 Method Not Allowed for GET, but that's OK)
        response = requests.get("http://localhost:8000/api/ask", timeout=10)
        print(f"GET /api/ask status: {response.status_code}")

        if response.status_code in [405, 422]:  # 405 = Method Not Allowed (good - endpoint exists), 422 = Validation Error (also good)
            print("/api/ask endpoint exists!")
            print("This is expected - GET requests are not allowed, only POST")
            print()
        elif response.status_code == 404:
            print("/api/ask endpoint not found")
            return False
        else:
            print(f"Unexpected status for GET: {response.status_code}")
            print(f"Response: {response.text}")

        # Test with a proper POST request (without authentication to see if route exists)
        headers = {
            'Content-Type': 'application/json'
        }
        data = {
            "question": "test",
            "mode": "full_book"
        }

        response = requests.post(
            "http://localhost:8000/api/ask",
            headers=headers,
            data=json.dumps(data),
            timeout=10
        )

        print(f"POST /api/ask status: {response.status_code}")

        if response.status_code in [401, 403, 422]:  # Unauthorized/Forbidden/Validation - good, means endpoint exists
            print("/api/ask endpoint is accessible (requires authentication or has validation)")
            print("This is expected - the endpoint exists but needs proper authentication")
            return True
        elif response.status_code == 404:
            print("404 Not Found - endpoint does not exist")
            return False
        else:
            print(f"Unexpected response from /api/ask: {response.status_code}")
            print(f"Response: {response.text}")
            return False

    except requests.exceptions.ConnectionError:
        print("Cannot connect to API - is the server running on http://localhost:8000?")
        return False
    except Exception as e:
        print(f"Error testing API: {e}")
        return False

if __name__ == "__main__":
    print("API /api/ask Endpoint Test")
    print("=" * 30)

    success = test_api_ask()

    if success:
        print("\nAPI ENDPOINT: ACCESSIBLE")
        print("The chatbot should now be able to reach the /api/ask endpoint!")
        print("The '404 Not Found' error should be resolved!")
    else:
        print("\nAPI ENDPOINT: NOT ACCESSIBLE")
        print("The /api/ask endpoint is still not accessible")