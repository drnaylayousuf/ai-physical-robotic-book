#!/usr/bin/env python3
"""
Test script to verify the backend API is working
"""
import requests
import time

def test_api_connection():
    """Test if the backend API is accessible"""
    print("Testing API connection...")
    print("Checking if backend is running on http://localhost:8000")
    print()

    try:
        # Test the root endpoint
        response = requests.get("http://localhost:8000/", timeout=10)
        print(f"Root endpoint status: {response.status_code}")

        if response.status_code == 200:
            data = response.json()
            print(f"Response: {data}")
            print("API is accessible!")
            print()

            # Test health endpoint
            health_response = requests.get("http://localhost:8000/health", timeout=10)
            print(f"Health endpoint status: {health_response.status_code}")
            if health_response.status_code == 200:
                health_data = health_response.json()
                print(f"Health: {health_data}")
                print("Health check passed!")
            else:
                print("Health check failed")

            return True
        else:
            print("API is not responding properly")
            return False

    except requests.exceptions.ConnectionError:
        print("Cannot connect to API - is the server running on http://localhost:8000?")
        print("Make sure you started the backend server with: python -m backend.main")
        return False
    except Exception as e:
        print(f"Error testing API: {e}")
        return False

if __name__ == "__main__":
    print("API Connection Test")
    print("=" * 30)

    success = test_api_connection()

    if success:
        print("\nAPI CONNECTION: SUCCESSFUL")
        print("The chatbot should now be able to connect to the backend!")
        print("The 'Failed to fetch' error should be resolved!")
    else:
        print("\nAPI CONNECTION: FAILED")
        print("Make sure the backend server is running on http://localhost:8000")