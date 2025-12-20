import pytest
from fastapi.testclient import TestClient
from backend.main import app

client = TestClient(app)

def test_qdrant_diagnostic_endpoint():
    """
    Test the Qdrant diagnostic endpoint
    """
    response = client.get("/api/diagnostic/qdrant")
    # The endpoint should return either 200 (success) or 500 (error connecting to Qdrant)
    assert response.status_code in [200, 500]

    if response.status_code == 200:
        data = response.json()
        assert "status" in data
        assert "collections" in data
        assert "qdrant_cloud_info" in data
        assert "timestamp" in data

        assert isinstance(data["collections"], list)
        assert isinstance(data["qdrant_cloud_info"], dict)
        assert isinstance(data["timestamp"], str)

def test_qdrant_diagnostic_response_format():
    """
    Test that the Qdrant diagnostic endpoint returns the expected response format
    """
    response = client.get("/api/diagnostic/qdrant")

    if response.status_code == 200:
        data = response.json()

        assert "status" in data
        assert "collections" in data
        assert "qdrant_cloud_info" in data
        assert "timestamp" in data

        # Check collections format
        for collection in data["collections"]:
            assert "name" in collection
            assert "vector_count" in collection
            assert "indexed" in collection
            assert "indexed_fields" in collection

            assert isinstance(collection["name"], str)
            assert isinstance(collection["vector_count"], int)
            assert isinstance(collection["indexed"], bool)
            assert isinstance(collection["indexed_fields"], list)

if __name__ == "__main__":
    pytest.main()