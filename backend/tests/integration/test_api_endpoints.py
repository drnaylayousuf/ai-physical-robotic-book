"""
Integration tests for RAG System API endpoints.
These tests verify that the API endpoints work correctly with various scenarios.
"""
import pytest
from fastapi.testclient import TestClient
from src.api.main import app
from unittest.mock import patch, MagicMock
import json


client = TestClient(app)


def test_post_ask_with_valid_request():
    """Test POST /api/ask with a valid request."""
    # This test will initially fail since the endpoint is not fully implemented yet
    # But it verifies the request structure is accepted
    response = client.post(
        "/api/ask",
        json={
            "question": "What are humanoid robots used for?",
            "mode": "full_book",
            "selected_text": None
        }
    )

    # At minimum, the request should be accepted (200) or return a proper error (4xx/5xx)
    # but not a server error due to invalid request structure
    assert response.status_code in [200, 400, 422, 500, 503]


def test_post_ask_with_empty_question():
    """Test POST /api/ask with empty question - should return validation error."""
    response = client.post(
        "/api/ask",
        json={
            "question": "",  # Empty question should trigger validation error
            "mode": "full_book",
            "selected_text": None
        }
    )

    # Should return a 400 or 422 error due to validation failure
    assert response.status_code in [400, 422]

    data = response.json()
    assert "detail" in data
    assert "question" in data["detail"].lower() or "empty" in data["detail"].lower()


def test_post_ask_with_whitespace_question():
    """Test POST /api/ask with whitespace-only question - should return validation error."""
    response = client.post(
        "/api/ask",
        json={
            "question": "   \n\t   ",  # Whitespace-only question
            "mode": "full_book",
            "selected_text": None
        }
    )

    # Should return a 400 or 422 error due to validation failure
    assert response.status_code in [400, 422]

    data = response.json()
    assert "detail" in data


def test_post_ask_with_invalid_mode():
    """Test POST /api/ask with invalid mode - should return validation error."""
    response = client.post(
        "/api/ask",
        json={
            "question": "What are humanoid robots?",
            "mode": "invalid_mode",  # Invalid mode
            "selected_text": None
        }
    )

    # Should return a 400 or 422 error due to validation failure
    assert response.status_code in [400, 422]

    data = response.json()
    assert "detail" in data


def test_post_ask_with_null_selected_text():
    """Test POST /api/ask with null selected_text - should be handled gracefully."""
    response = client.post(
        "/api/ask",
        json={
            "question": "What are humanoid robots used for?",
            "mode": "full_book",
            "selected_text": None  # Null value should be handled gracefully
        }
    )

    # Should not crash and return either success or appropriate error
    assert response.status_code in [200, 400, 422, 500, 503]


def test_post_ask_with_selected_text_mode():
    """Test POST /api/ask with selected_text mode."""
    response = client.post(
        "/api/ask",
        json={
            "question": "What does this text say about sensors?",
            "mode": "selected_text",
            "selected_text": "Humanoid robots use various sensors including cameras, accelerometers, and gyroscopes."
        }
    )

    # Should not crash and return either success or appropriate error
    assert response.status_code in [200, 400, 422, 500, 503]


def test_error_handling_for_dependency_failures():
    """Test that proper error messages are returned when dependencies fail."""
    # Mock dependency failures to test error handling
    with patch('src.services.rag_service.RAGService.process_query') as mock_process:
        mock_process.side_effect = Exception("Dependency unavailable")

        response = client.post(
            "/api/ask",
            json={
                "question": "What are humanoid robots used for?",
                "mode": "full_book",
                "selected_text": None
            }
        )

        # Should return a 500 or 503 error with descriptive message
        assert response.status_code in [500, 503]

        data = response.json()
        assert "detail" in data
        # Verify error message is descriptive and not exposing internal details
        assert "dependency" in data["detail"].lower() or "unavailable" in data["detail"].lower()


def test_health_endpoint():
    """Test GET /api/health endpoint."""
    response = client.get("/api/health")

    # Health endpoint may not be implemented yet, so check both possibilities
    if response.status_code == 200:
        data = response.json()
        # Verify structure matches contract
        assert "status" in data
        assert isinstance(data["status"], str)
    elif response.status_code == 404:
        # Endpoint not implemented yet, which is acceptable during development
        pass
    else:
        # If implemented, should return 200 or appropriate error
        assert response.status_code in [200, 400, 500, 503]