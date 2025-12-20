#!/usr/bin/env python3
"""
Script to run the server and ingestion in a coordinated way.
"""

import subprocess
import time
import requests
import sys
import os
from datetime import timedelta
from backend.config.settings import settings
from backend.utils.auth import create_access_token, create_token_data
from backend.models.database import SessionLocal
from backend.models.user import User

def start_server():
    """Start the backend server."""
    print("Starting backend server...")

    # Start the server in a subprocess
    server_process = subprocess.Popen([
        sys.executable, "-c",
        "import uvicorn; from backend.main import app; uvicorn.run(app, host='0.0.0.0', port=8001)"
    ], cwd=os.getcwd())

    # Wait a bit for the server to start
    time.sleep(5)

    return server_process

def generate_admin_token():
    """Generate a JWT token for the admin user."""
    # Get admin user from database
    db = SessionLocal()
    try:
        admin_user = db.query(User).filter(User.username == "admin").first()
        if not admin_user:
            raise Exception("Admin user not found in database")

        # Create token data
        token_data = create_token_data(admin_user.username, admin_user.role)

        # Create access token
        token = create_access_token(
            data=token_data,
            expires_delta=timedelta(minutes=60)  # Token valid for 1 hour
        )

        print(f"Generated admin token for user {admin_user.username}")
        return token

    finally:
        db.close()

def wait_for_server():
    """Wait for the server to be ready."""
    print("Waiting for server to be ready...")
    for i in range(20):  # Wait up to 20 seconds
        try:
            response = requests.get("http://localhost:8001/api/health", timeout=5)
            if response.status_code == 200:
                print("Server is ready!")
                return True
        except:
            pass
        print(f"Waiting for server... ({i+1}/20)")
        time.sleep(1)
    return False

def run_ingestion():
    """Run the ingestion process."""
    print("Starting ingestion process...")

    # Generate admin token
    admin_token = generate_admin_token()
    print(f"Token: {admin_token[:20]}...")  # Show first 20 chars for verification

    # Call ingestion endpoint
    headers = {
        "Authorization": f"Bearer {admin_token}",
        "Content-Type": "application/json"
    }

    import os
    # Request to ingest content from the doc directory using absolute path
    # The script runs from main directory, so doc is in ./doc
    source_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "doc"))
    ingestion_data = {
        "source_path": source_path,
        "collection_name": "book_chunks"
    }

    try:
        print("Calling ingestion endpoint...")
        response = requests.post(
            "http://localhost:8001/api/ingest",
            headers=headers,
            json=ingestion_data,
            timeout=300  # 5 minute timeout for potentially large ingestion
        )

        print(f"Ingestion response status: {response.status_code}")
        print(f"Ingestion response: {response.text}")

        if response.status_code == 200:
            print("Ingestion completed successfully!")
            return True
        else:
            print(f"Ingestion failed with status {response.status_code}")
            return False

    except requests.exceptions.RequestException as e:
        print(f"Error calling ingestion endpoint: {e}")
        return False

def test_qa():
    """Test the question answering functionality."""
    print("\nTesting question answering...")

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
            print(f"QA Response: {response_data.get('response', 'No response field')[:200]}...")
            return True
        else:
            print(f"QA failed with status {response.status_code}")
            print(f"Response: {response.text}")
            return False
    except Exception as e:
        print(f"Error testing QA: {e}")
        return False

def main():
    """Main function to coordinate server start and ingestion."""
    server_process = None

    try:
        # Start the server
        server_process = start_server()

        # Wait for the server to be ready
        if not wait_for_server():
            print("Server failed to start properly")
            return False

        # Run the ingestion
        ingestion_success = run_ingestion()

        if ingestion_success:
            print("\nIngestion completed successfully!")

            # Test the QA functionality
            qa_success = test_qa()

            if qa_success:
                print("\nSUCCESS: The system is working properly!")
                print("   - Book content has been ingested")
                print("   - Question answering is functional")
            else:
                print("\nWARNING: Ingestion worked but QA test failed")
        else:
            print("\nFAILED: Ingestion did not complete successfully")

    except Exception as e:
        print(f"Error in main process: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Clean up: terminate the server process
        if server_process:
            print("\nShutting down server...")
            server_process.terminate()
            server_process.wait()
            print("Server stopped.")

if __name__ == "__main__":
    main()