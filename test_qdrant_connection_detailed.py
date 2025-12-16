#!/usr/bin/env python3
"""
Qdrant Connection Test Script
This script will test the connection to your Qdrant instance and verify that data operations work.
"""
import asyncio
import os
import sys
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct, Distance, VectorParams

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

# Get Qdrant configuration from environment
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_chunks")

async def test_qdrant_connection():
    print("🔍 Testing Qdrant Connection...")
    print(f"URL: {QDRANT_URL}")
    print(f"Collection: {QDRANT_COLLECTION_NAME}")
    print()

    try:
        # Initialize Qdrant client
        if QDRANT_API_KEY:
            client = QdrantClient(
                url=QDRANT_URL,
                api_key=QDRANT_API_KEY,
                timeout=10
            )
        else:
            client = QdrantClient(
                url=QDRANT_URL,
                timeout=10
            )

        print("✅ Successfully connected to Qdrant!")
        print()

        # Test 1: Get collections list
        print("📋 Testing: Get collections...")
        collections = client.get_collections()
        print(f"✅ Found {len(collections.collections)} collection(s)")
        for collection in collections.collections:
            print(f"   - {collection.name} (vectors: {collection.vector_size}, points: {collection.points_count})")
        print()

        # Test 2: Check if our collection exists or create it
        print(f"🔍 Testing: Check/Create collection '{QDRANT_COLLECTION_NAME}'...")
        try:
            collection_info = client.get_collection(QDRANT_COLLECTION_NAME)
            print(f"✅ Collection '{QDRANT_COLLECTION_NAME}' exists!")
            print(f"   - Vector size: {collection_info.config.params.size}")
            print(f"   - Distance: {collection_info.config.params.distance}")
            print(f"   - Points count: {collection_info.points_count}")
        except Exception as e:
            print(f"⚠️  Collection '{QDRANT_COLLECTION_NAME}' doesn't exist, creating it...")
            client.create_collection(
                collection_name=QDRANT_COLLECTION_NAME,
                vectors_config=VectorParams(size=384, distance=Distance.COSINE)  # Using 384 dims for compatibility
            )
            print(f"✅ Collection '{QDRANT_COLLECTION_NAME}' created successfully!")
        print()

        # Test 3: Insert a test point
        print("💾 Testing: Insert a test point...")
        test_point = PointStruct(
            id=1,
            vector=[0.1] * 384,  # Simple test vector
            payload={
                "test": True,
                "content": "This is a test document for humanoid robotics book",
                "title": "Test Document"
            }
        )

        client.upsert(
            collection_name=QDRANT_COLLECTION_NAME,
            points=[test_point]
        )
        print("✅ Test point inserted successfully!")
        print()

        # Test 4: Search for the test point
        print("🔍 Testing: Search for inserted point...")
        search_results = client.search(
            collection_name=QDRANT_COLLECTION_NAME,
            query_vector=[0.1] * 384,
            limit=1,
            with_payload=True
        )

        if search_results:
            result = search_results[0]
            print(f"✅ Search successful! Found {len(search_results)} result(s)")
            print(f"   - ID: {result.id}")
            print(f"   - Score: {result.score:.4f}")
            print(f"   - Content: {result.payload.get('content', 'N/A')}")
        else:
            print("⚠️  No results found in search")
        print()

        # Test 5: Count points in the collection
        print("📊 Testing: Count points in collection...")
        count = client.count(
            collection_name=QDRANT_COLLECTION_NAME,
            exact=True
        )
        print(f"✅ Collection contains {count.count} point(s)")
        print()

        # Test 6: List all points (first few)
        print("📋 Testing: List points in collection...")
        points = client.scroll(
            collection_name=QDRANT_COLLECTION_NAME,
            limit=5,  # Get first 5 points
            with_payload=True,
            with_vectors=False
        )
        points_list, next_page = points
        print(f"✅ Retrieved {len(points_list)} point(s) from collection")
        for i, point in enumerate(points_list):
            print(f"   {i+1}. ID: {point.id}, Payload keys: {list(point.payload.keys())}")
        print()

        # Clean up: Remove the test point
        print("🧹 Testing: Clean up test point...")
        client.delete(
            collection_name=QDRANT_COLLECTION_NAME,
            points_selector=[1]  # Delete our test point with ID 1
        )
        print("✅ Test point cleaned up!")
        print()

        print("🎉 All Qdrant tests passed! Your Qdrant connection is working properly.")
        print("💡 The chatbot will now be able to store and retrieve book content in Qdrant.")

        return True

    except Exception as e:
        print(f"❌ Error during Qdrant testing: {e}")
        print()
        print("🔧 Troubleshooting Tips:")
        print("1. Verify your QDRANT_URL is correct and accessible")
        print("2. Check that your QDRANT_API_KEY is valid")
        print("3. Ensure your network/firewall allows connections to Qdrant")
        print("4. If using a cloud instance, verify the instance is running")
        print("5. Check that the Qdrant service is properly configured")
        return False

if __name__ == "__main__":
    print("🚀 Starting Qdrant Connection Test")
    print("=" * 50)

    success = asyncio.run(test_qdrant_connection())

    print("=" * 50)
    if success:
        print("✅ QDRANT CONNECTION TEST: PASSED")
        print("💡 Your Qdrant instance is ready to use with the chatbot!")
    else:
        print("❌ QDRANT CONNECTION TEST: FAILED")
        print("⚠️  Please check the error messages above and fix the connection issues.")

    sys.exit(0 if success else 1)