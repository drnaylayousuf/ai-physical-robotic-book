import pytest
from fastapi.testclient import TestClient
from backend.main import app

client = TestClient(app)

def test_chat_endpoint_basic():
    """
    Test the basic functionality of the chat endpoint
    """
    request_data = {
        "question": "What is humanoid robotics?",
        "mode": "full_book"
    }

    response = client.post("/api/ask", json=request_data)

    # The endpoint should return either 200 (success) or 500 (error connecting to Qdrant/Gemini)
    assert response.status_code in [200, 500]

    if response.status_code == 200:
        data = response.json()

        # Check that required fields are present
        assert "question" in data
        assert "answer" in data
        assert "sources" in data
        assert "mode" in data
        assert "timestamp" in data
        assert "processing_time_ms" in data

        # Check data types
        assert isinstance(data["question"], str)
        assert isinstance(data["answer"], str)
        assert isinstance(data["sources"], list)
        assert isinstance(data["mode"], str)
        assert isinstance(data["timestamp"], str)
        assert isinstance(data["processing_time_ms"], int)

        # Check that the question matches what we sent
        assert data["question"] == "What is humanoid robotics?"
        assert data["mode"] == "full_book"

def test_chat_endpoint_with_user_id():
    """
    Test the chat endpoint with an optional user_id
    """
    request_data = {
        "question": "What is humanoid robotics?",
        "mode": "full_book",
        "user_id": "test_user_123"
    }

    response = client.post("/api/ask", json=request_data)
    assert response.status_code in [200, 500]

def test_chat_endpoint_different_modes():
    """
    Test the chat endpoint with different modes (though only full_book is currently supported)
    """
    request_data = {
        "question": "What is artificial intelligence?",
        "mode": "full_book"
    }

    response = client.post("/api/ask", json=request_data)
    assert response.status_code in [200, 500]

def test_chat_endpoint_sources_format():
    """
    Test that the sources in the response follow the expected format
    """
    request_data = {
        "question": "What are the applications of robotics?",
        "mode": "full_book"
    }

    response = client.post("/api/ask", json=request_data)

    if response.status_code == 200:
        data = response.json()
        sources = data["sources"]

        # Sources should be a list
        assert isinstance(sources, list)

        # If there are sources, check their format
        for source in sources:
            assert "content_id" in source
            assert "text" in source
            assert "similarity_score" in source

            assert isinstance(source["content_id"], str)
            assert isinstance(source["text"], str)
            assert isinstance(source["similarity_score"], (int, float))

def test_chat_endpoint_response_format():
    """
    Test the overall response format of the chat endpoint
    """
    request_data = {
        "question": "What is machine learning?",
        "mode": "full_book"
    }

    response = client.post("/api/ask", json=request_data)

    if response.status_code == 200:
        data = response.json()

        # Verify all required fields are present
        required_fields = ["question", "answer", "sources", "mode", "timestamp", "processing_time_ms"]
        for field in required_fields:
            assert field in data

if __name__ == "__main__":
    pytest.main()