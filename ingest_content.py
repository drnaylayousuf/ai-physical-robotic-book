#!/usr/bin/env python3
"""
Simple script to ingest book content into the existing server.
"""

import requests
import json
from datetime import timedelta
from backend.config.settings import settings
from backend.utils.auth import create_access_token, create_token_data
from backend.models.database import SessionLocal
from backend.models.user import User

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

def call_ingestion_endpoint(token):
    """Call the ingestion endpoint to load book content."""
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json"
    }

    import os
    # Request to ingest content from the doc directory using absolute path
    source_path = os.path.abspath("doc")
    ingestion_data = {
        "source_path": source_path,
        "collection_name": "book_chunks"
    }

    try:
        print(f"Calling ingestion endpoint with path: {source_path}")
        response = requests.post(
            "http://localhost:8000/api/ingest",  # Use port 8000 instead of 8001
            headers=headers,
            json=ingestion_data,
            timeout=300  # 5 minute timeout for potentially large ingestion
        )

        print(f"Ingestion response status: {response.status_code}")
        print(f"Ingestion response: {response.text}")

        if response.status_code == 200:
            print("Ingestion started successfully!")
            return True
        else:
            print(f"Ingestion failed with status {response.status_code}")
            return False

    except requests.exceptions.RequestException as e:
        print(f"Error calling ingestion endpoint: {e}")
        return False

def check_ingestion_status(token):
    """Check the status of the ingestion process."""
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json"
    }

    try:
        response = requests.get(
            "http://localhost:8000/api/ingest/status",
            headers=headers,
            timeout=30
        )

        print(f"Ingestion status response: {response.status_code}")
        print(f"Status details: {response.text}")
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"Error checking ingestion status: {e}")
        return None

if __name__ == "__main__":
    print("Generating admin token...")
    admin_token = generate_admin_token()
    print(f"Token: {admin_token[:20]}...")  # Show first 20 chars for verification

    print("\nStarting content ingestion...")
    success = call_ingestion_endpoint(admin_token)

    if success:
        print("\nChecking ingestion status...")
        import time
        time.sleep(5)  # Wait a bit for processing to start
        status = check_ingestion_status(admin_token)
        print(f"\nIngestion status: {status}")
    else:
        print("Failed to start ingestion.")