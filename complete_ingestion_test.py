#!/usr/bin/env python3
"""
Script to properly ingest the book content and test the RAG functionality.
"""

import subprocess
import sys
import time
import requests
from datetime import timedelta
from backend.config.settings import settings
from backend.utils.auth import create_access_token, create_token_data
from backend.models.database import SessionLocal
from backend.models.user import User
import os

def start_server():
    """Start the backend server on port 8000."""
    # Start the server in a subprocess
    server_process = subprocess.Popen([
        sys.executable, '-c',
        'import uvicorn; from backend.main import app; uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")'
    ])
    return server_process

def generate_admin_token():
    """Generate a JWT token for the admin user."""
    db = SessionLocal()
    try:
        admin_user = db.query(User).filter(User.username == "admin").first()
        if not admin_user:
            raise Exception("Admin user not found in database")

        token_data = create_token_data(admin_user.username, admin_user.role)
        token = create_access_token(data=token_data, expires_delta=timedelta(minutes=60))
        return token
    finally:
        db.close()

def wait_for_server():
    """Wait for the server to be ready."""
    print("Waiting for server to be ready...")
    for i in range(20):  # Wait up to 20 seconds
        try:
            response = requests.get("http://localhost:8000/api/health", timeout=5)
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

    # Request to ingest content from the doc directory using absolute path
    source_path = os.path.abspath("doc")
    ingestion_data = {
        "source_path": source_path,
        "collection_name": "book_chunks"
    }

    try:
        print(f"Calling ingestion endpoint with path: {source_path}")
        response = requests.post(
            "http://localhost:8000/api/ingest",
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

def test_diagnostic():
    """Test the diagnostic endpoint."""
    try:
        response = requests.get('http://localhost:8000/api/diagnostic/qdrant')
        print('Diagnostic response:', response.text)

        if response.status_code == 200:
            data = response.json()
            total_chunks = data.get('total_chunks', 0)
            print(f'Total chunks in Qdrant: {total_chunks}')
            return total_chunks > 0
    except Exception as e:
        print(f'Diagnostic test error: {e}')

    return False

def test_qa():
    """Test the question answering functionality."""
    try:
        response = requests.post('http://localhost:8000/api/ask',
                               json={'question': 'What is humanoid robotics?', 'mode': 'full_book'},
                               headers={'Content-Type': 'application/json'})
        print('QA test response:', response.text)

        if response.status_code == 200:
            data = response.json()
            answer = data.get('response', '')
            sources = data.get('sources', [])
            print(f'QA Answer length: {len(answer)}')
            print(f'QA Sources count: {len(sources)}')

            # Check if it's still the fallback response
            if "The book does not provide details about this topic" in answer:
                print("QA test failed - still showing fallback response")
                return False
            else:
                print("QA test successful - got meaningful response")
                return True
    except Exception as e:
        print(f'QA test error: {e}')

    return False

def main():
    """Main function to coordinate server start and ingestion."""
    server_process = None

    try:
        # Start the server
        print("Starting backend server...")
        server_process = start_server()

        # Wait for the server to be ready
        if not wait_for_server():
            print("Server failed to start properly")
            return False

        # Run the ingestion
        ingestion_success = run_ingestion()

        if ingestion_success:
            print("\nIngestion completed successfully!")

            # Wait a moment for the data to be fully processed
            time.sleep(3)

            # Test the diagnostic
            diagnostic_success = test_diagnostic()

            if diagnostic_success:
                # Test the QA functionality
                qa_success = test_qa()

                if qa_success:
                    print("\nSUCCESS: The system is working properly!")
                    print("   - Book content has been ingested")
                    print("   - Diagnostic shows proper chunk count")
                    print("   - Question answering is functional")
                    return True
                else:
                    print("\nWARNING: Ingestion and diagnostic worked but QA test failed")
            else:
                print("\nWARNING: Ingestion worked but diagnostic shows 0 chunks")
                print("This suggests the content might be stored in memory rather than persistent storage")
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
            try:
                server_process.wait(timeout=5)  # Wait up to 5 seconds
            except subprocess.TimeoutExpired:
                server_process.kill()  # Force kill if it doesn't terminate gracefully
            print("Server stopped.")

    return False

if __name__ == "__main__":
    success = main()
    if success:
        print("\nAll tests passed! The chatbot should now work properly.")
    else:
        print("\nSome tests failed. The chatbot may not work properly.")