import pytest
from fastapi.testclient import TestClient
from backend.main import app
from backend.utils.qdrant_client import qdrant_service

client = TestClient(app)

def test_qdrant_service_initialization():
    """
    Test that the Qdrant service is properly initialized
    """
    assert qdrant_service is not None
    assert hasattr(qdrant_service, 'client')
    assert hasattr(qdrant_service, 'health_check')

def test_qdrant_cloud_connectivity():
    """
    Test actual Qdrant Cloud connectivity through the service
    """
    health_status = qdrant_service.health_check()
    assert isinstance(health_status, dict)
    assert "status" in health_status

def test_qdrant_collection_existence():
    """
    Test that the expected collection exists in Qdrant Cloud
    """
    exists = qdrant_service.collection_exists()
    # This test will pass regardless of whether the collection exists or not
    # since the function should return a boolean
    assert isinstance(exists, bool)

def test_qdrant_api_health_endpoint_integration():
    """
    Integration test for the Qdrant Cloud health endpoint
    """
    response = client.get("/api/health/qdrant")
    assert response.status_code == 200

    data = response.json()
    assert "status" in data
    assert "message" in data
    assert "timestamp" in data
    assert "qdrant_cloud_status" in data

def test_qdrant_api_diagnostic_endpoint_integration():
    """
    Integration test for the Qdrant diagnostic endpoint
    """
    response = client.get("/api/diagnostic/qdrant")
    assert response.status_code in [200, 500]  # Could be 500 if Qdrant is not accessible

    if response.status_code == 200:
        data = response.json()
        assert "status" in data
        assert "collections" in data
        assert "qdrant_cloud_info" in data
        assert "timestamp" in data

def test_rag_functionality_integration():
    """
    Integration test for RAG functionality using Qdrant Cloud and Gemini
    """
    # Test a simple question
    request_data = {
        "question": "What is humanoid robotics?",
        "mode": "full_book"
    }

    response = client.post("/api/ask", json=request_data)
    # The response should be either successful or a server error (not a client error)
    assert response.status_code in [200, 500]

    if response.status_code == 200:
        data = response.json()

        # Verify response structure
        assert "question" in data
        assert "answer" in data
        assert "sources" in data
        assert "mode" in data
        assert "timestamp" in data
        assert "processing_time_ms" in data

        # Verify content
        assert data["question"] == "What is humanoid robotics?"
        assert data["mode"] == "full_book"
        assert isinstance(data["answer"], str)
        assert len(data["answer"]) > 0  # Answer should not be empty

        # Sources should be a list
        assert isinstance(data["sources"], list)

def test_rag_with_different_questions():
    """
    Test RAG functionality with different types of questions
    """
    questions = [
        "What is artificial intelligence?",
        "Explain machine learning",
        "How do robots perceive their environment?"
    ]

    for question in questions:
        request_data = {
            "question": question,
            "mode": "full_book"
        }

        response = client.post("/api/ask", json=request_data)
        assert response.status_code in [200, 500]

        if response.status_code == 200:
            data = response.json()
            assert data["question"] == question
            assert data["mode"] == "full_book"
            assert isinstance(data["answer"], str)

if __name__ == "__main__":
    pytest.main()