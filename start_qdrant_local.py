#!/usr/bin/env python3
"""
Script to start a local Qdrant instance for development
"""
import asyncio
from qdrant_client import QdrantClient
import time
import threading

def start_local_qdrant():
    """Start a local Qdrant instance using local storage"""
    print("Starting local Qdrant instance...")

    # Create a local Qdrant client using local storage
    import tempfile
    import os
    # Use a fixed location so the backend can access the same data
    qdrant_path = "./qdrant_storage"
    os.makedirs(qdrant_path, exist_ok=True)

    client = QdrantClient(path=qdrant_path)  # Local persistent storage
    print(f"Local Qdrant instance is ready at: {qdrant_path}")

    print("Collections available:")

    try:
        collections = client.get_collections()
        for collection in collections.collections:
            print(f"  - {collection.name} (points: {collection.points_count})")
    except Exception as e:
        print(f"  - No collections yet (this is normal): {e}")

    # Keep the instance running
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down local Qdrant instance...")
        return

if __name__ == "__main__":
    start_local_qdrant()