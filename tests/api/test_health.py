import pytest
from fastapi.testclient import TestClient
from backend.main import app

client = TestClient(app)

def test_basic_health_endpoint():
    """
    Test the basic health endpoint
    """
    response = client.get("/api/health")
    assert response.status_code == 200
    assert "status" in response.json()
    assert "timestamp" in response.json()
    assert response.json()["status"] == "healthy"

def test_qdrant_cloud_health_endpoint():
    """
    Test the Qdrant Cloud health endpoint
    """
    response = client.get("/api/health/qdrant")
    assert response.status_code == 200
    assert "status" in response.json()
    assert "message" in response.json()
    assert "timestamp" in response.json()
    assert "qdrant_cloud_status" in response.json()

    # The status should be either "healthy" or "unhealthy" based on actual connectivity
    assert response.json()["status"] in ["healthy", "unhealthy"]

def test_qdrant_cloud_health_response_format():
    """
    Test that the Qdrant Cloud health endpoint returns the expected response format
    """
    response = client.get("/api/health/qdrant")
    data = response.json()

    assert "status" in data
    assert "message" in data
    assert "timestamp" in data
    assert "qdrant_cloud_status" in data

    assert isinstance(data["status"], str)
    assert isinstance(data["message"], str)
    assert isinstance(data["timestamp"], str)
    assert isinstance(data["qdrant_cloud_status"], str)

if __name__ == "__main__":
    pytest.main()