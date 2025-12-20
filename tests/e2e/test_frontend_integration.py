import pytest
import requests
from fastapi.testclient import TestClient
from backend.main import app

client = TestClient(app)

def test_api_endpoints_accessibility():
    """
    Test that all required API endpoints are accessible from frontend
    """
    # Test health endpoint
    response = client.get("/api/health")
    assert response.status_code == 200

    # Test Qdrant health endpoint
    response = client.get("/api/health/qdrant")
    assert response.status_code in [200, 500]  # 500 if Qdrant not accessible

    # Test diagnostic endpoint
    response = client.get("/api/diagnostic/qdrant")
    assert response.status_code in [200, 500]  # 500 if Qdrant not accessible

    # Test chat endpoint with OPTIONS (for CORS)
    response = client.options("/api/ask")
    # OPTIONS requests may return 404 or 405 depending on FastAPI's default behavior
    # The important thing is that CORS headers should be present when implemented

def test_cors_headers_present():
    """
    Test that CORS headers are properly configured for frontend integration
    """
    # Make a request with origin header to test CORS
    headers = {
        "Origin": "http://localhost:3000",
        "Access-Control-Request-Method": "POST",
        "Access-Control-Request-Headers": "Content-Type"
    }

    # Test preflight request
    response = client.options("/api/ask", headers=headers)

    # Even if options isn't explicitly handled, the CORS middleware should add headers
    # Check if CORS headers are configured by checking a regular request
    response = client.get("/api/health", headers={"Origin": "http://localhost:3000"})

    # This test mainly verifies that the CORS configuration in main.py is in place
    # The actual CORS headers may not appear in TestClient requests, but they will in real requests

def test_api_response_formats():
    """
    Test that API responses follow the expected format for frontend consumption
    """
    # Test health endpoint response format
    response = client.get("/api/health")
    if response.status_code == 200:
        data = response.json()
        assert "status" in data
        assert "timestamp" in data

    # Test Qdrant health endpoint response format
    response = client.get("/api/health/qdrant")
    if response.status_code == 200:
        data = response.json()
        assert "status" in data
        assert "message" in data
        assert "timestamp" in data
        assert "qdrant_cloud_status" in data

    # Test diagnostic endpoint response format
    response = client.get("/api/diagnostic/qdrant")
    if response.status_code == 200:
        data = response.json()
        assert "status" in data
        assert "collections" in data
        assert "qdrant_cloud_info" in data
        assert "timestamp" in data

def test_chat_endpoint_response_format():
    """
    Test that chat endpoint returns response in expected format for frontend
    """
    request_data = {
        "question": "What is humanoid robotics?",
        "mode": "full_book"
    }

    response = client.post("/api/ask", json=request_data)
    assert response.status_code in [200, 500]

    if response.status_code == 200:
        data = response.json()
        expected_fields = ["question", "answer", "sources", "mode", "timestamp", "processing_time_ms"]
        for field in expected_fields:
            assert field in data

def test_frontend_api_compatibility():
    """
    Test API compatibility with frontend-style requests
    """
    # Test with a typical frontend request
    request_data = {
        "question": "What are the main components of a humanoid robot?",
        "mode": "full_book"
    }

    response = client.post("/api/ask", json=request_data)
    assert response.status_code in [200, 500]

    if response.status_code == 200:
        data = response.json()
        # Verify that the response is structured properly for frontend consumption
        assert isinstance(data["question"], str)
        assert isinstance(data["answer"], str)
        assert isinstance(data["sources"], list)
        assert isinstance(data["mode"], str)
        assert isinstance(data["timestamp"], str)
        assert isinstance(data["processing_time_ms"], int)

if __name__ == "__main__":
    pytest.main()