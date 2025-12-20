#!/usr/bin/env python3
"""
Diagnostic script to check if the book content was properly ingested into Qdrant.
"""

import requests
import json

def check_qdrant_status():
    """Check the status of Qdrant collections and content."""

    url = "http://localhost:8000/api/diagnostic/qdrant"

    print("Checking Qdrant diagnostic information...")
    print(f"Sending request to: {url}")

    try:
        response = requests.get(url, timeout=30)
        print(f"Response Status Code: {response.status_code}")

        if response.status_code == 200:
            response_data = response.json()
            print(f"Response Data: {json.dumps(response_data, indent=2)}")

            print("\n" + "="*50)
            print("QDRANT DIAGNOSTIC RESULTS:")
            print(f"Collections: {response_data.get('collections', [])}")
            print(f"Total Chunks: {response_data.get('total_chunks', 0)}")
            print(f"Status: {response_data.get('status', 'unknown')}")

            sample_chunks = response_data.get('sample_chunks', [])
            if sample_chunks:
                print("\nSample Chunks:")
                for i, chunk in enumerate(sample_chunks, 1):
                    print(f"  {i}. ID: {chunk.get('chunk_id')}")
                    print(f"     Content preview: {chunk.get('content', '')[:100]}...")
                    print(f"     Source: {chunk.get('source', 'Unknown')}")
            else:
                print("\nNo sample chunks found - content may not be properly ingested.")

            print("="*50)

            if response_data.get('total_chunks', 0) > 0:
                print("SUCCESS: Book content has been ingested into Qdrant!")
                print(f"   Found {response_data.get('total_chunks', 0)} chunks in the database.")
            else:
                print("ISSUE: No chunks found in Qdrant. Content may not have been properly ingested.")

        else:
            print(f"Request failed with status code: {response.status_code}")
            print(f"Response text: {response.text}")

    except requests.exceptions.ConnectionError:
        print("ERROR: Cannot connect to the server. Is it running on http://localhost:8000?")
    except requests.exceptions.Timeout:
        print("ERROR: Request timed out.")
    except requests.exceptions.RequestException as e:
        print(f"ERROR: Request failed with exception: {e}")
    except json.JSONDecodeError:
        print(f"Response text (not JSON): {response.text}")

if __name__ == "__main__":
    check_qdrant_status()