import requests
import json
import sys
import os

# Add backend to path
sys.path.append('backend')

def test_api():
    # Test the /api/ask endpoint
    url = "http://localhost:8000/api/ask"

    # Test payload
    payload = {
        "question": "What is humanoid robotics?",
        "mode": "full_book"
    }

    headers = {
        "Content-Type": "application/json"
    }

    try:
        print("Testing API endpoint...")
        response = requests.post(url, json=payload, headers=headers)

        print(f"Status Code: {response.status_code}")
        print(f"Response Headers: {dict(response.headers)}")

        if response.status_code == 200:
            try:
                data = response.json()
                print(f"Response JSON: {json.dumps(data, indent=2)}")
            except:
                print(f"Response Text: {response.text}")
        else:
            print(f"Error Response: {response.text}")

    except requests.exceptions.ConnectionError:
        print("Connection Error: Backend server is not running on http://localhost:8000")
        print("Please start the backend server first using: python start_backend.py")
    except Exception as e:
        print(f"Error occurred: {e}")

if __name__ == "__main__":
    test_api()