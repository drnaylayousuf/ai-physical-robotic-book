import pytest
from fastapi.testclient import TestClient
from backend.main import app
from backend.models.database import SessionLocal, engine, Base
from backend.utils.auth import create_access_token, create_token_data


@pytest.fixture
def client():
    """Create a test client for the API"""
    with TestClient(app) as client:
        yield client


@pytest.fixture(autouse=True)
def setup_and_teardown():
    """Set up and tear down the test database"""
    # Create tables
    Base.metadata.create_all(bind=engine)

    yield  # Run the test

    # Clean up
    Base.metadata.drop_all(bind=engine)


def test_complete_chat_flow(client):
    """Test the complete chat flow: register, login, ask question"""
    # Register a user
    register_response = client.post("/auth/register", json={
        "username": "testuser",
        "email": "test@example.com",
        "password": "securepassword123"
    })
    assert register_response.status_code == 200

    # Login to get token
    login_response = client.post("/auth/login", json={
        "username": "testuser",
        "password": "securepassword123"
    })
    assert login_response.status_code == 200
    token = login_response.json()["access_token"]

    # Ask a question
    ask_response = client.post("/ask", json={
        "question": "What is Physical AI?",
        "mode": "full_book"
    }, headers={"Authorization": f"Bearer {token}"})

    # The response should be successful (even if it's a simulated response)
    assert ask_response.status_code == 200
    data = ask_response.json()
    assert "response" in data
    assert "sources" in data
    assert "references" in data


def test_chat_with_selected_text_mode(client):
    """Test chat functionality with selected text mode"""
    # Register a user
    client.post("/auth/register", json={
        "username": "testuser",
        "email": "test@example.com",
        "password": "securepassword123"
    })

    # Login to get token
    login_response = client.post("/auth/login", json={
        "username": "testuser",
        "password": "securepassword123"
    })
    token = login_response.json()["access_token"]

    # Ask a question in selected text mode
    ask_response = client.post("/ask", json={
        "question": "What does this text mean?",
        "mode": "selected_text",
        "selected_text": "This is a sample text for testing purposes."
    }, headers={"Authorization": f"Bearer {token}"})

    assert ask_response.status_code == 200
    data = ask_response.json()
    assert "response" in data
    assert "sources" in data
    assert "references" in data


def test_unauthenticated_chat_access(client):
    """Test that unauthenticated users cannot access chat"""
    response = client.post("/ask", json={
        "question": "What is Physical AI?",
        "mode": "full_book"
    })

    # Should return 401 Unauthorized
    assert response.status_code == 401


def test_health_endpoint(client):
    """Test the health endpoint"""
    response = client.get("/health")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"
    assert "timestamp" in data


def test_metadata_endpoint(client):
    """Test the metadata endpoint"""
    response = client.get("/metadata")
    assert response.status_code == 200
    data = response.json()
    assert "title" in data
    assert "chapters" in data
    assert isinstance(data["chapters"], list)


if __name__ == "__main__":
    pytest.main([__file__])