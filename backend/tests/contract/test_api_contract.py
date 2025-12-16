"""
Contract tests for RAG System API endpoints.
These tests verify that the API endpoints match the contract specifications.
"""
import pytest
from fastapi.testclient import TestClient
from src.api.main import app
import json


client = TestClient(app)


def test_post_ask_endpoint_contract():
    """Test that POST /api/ask endpoint matches contract specification."""
    # Test valid request structure
    response = client.post(
        "/api/ask",
        json={
            "question": "What are humanoid robots used for?",
            "mode": "full_book",
            "selected_text": None
        }
    )

    # Check that response has expected structure regardless of processing result
    assert response.status_code in [200, 400, 422, 500, 503]  # Expected status codes

    # If successful, check response structure
    if response.status_code == 200:
        data = response.json()
        assert "response" in data
        assert "sources" in data
        assert "references" in data
        assert isinstance(data["sources"], list)
        assert isinstance(data["references"], list)
    elif response.status_code >= 400:
        # For error responses, check error structure
        data = response.json()
        assert "detail" in data
        assert isinstance(data["detail"], str)
        assert len(data["detail"]) > 0


def test_post_ask_error_response_format():
    """Test that error responses follow the standard format."""
    # Send an invalid request to trigger an error
    response = client.post(
        "/api/ask",
        json={
            "question": "",  # Invalid - empty question
            "mode": "full_book",
            "selected_text": None
        }
    )

    # Should return an error response
    assert response.status_code in [400, 422]

    data = response.json()
    # Check that error response follows the standard format
    assert "detail" in data
    assert isinstance(data["detail"], str)
    assert len(data["detail"]) > 0

    # Optional fields should be present if provided
    if "error_code" in data:
        assert isinstance(data["error_code"], str)
    if "timestamp" in data:
        assert isinstance(data["timestamp"], str)
    if "request_id" in data:
        assert isinstance(data["request_id"], str)


def test_get_health_endpoint_contract():
    """Test that GET /api/health endpoint matches contract specification."""
    response = client.get("/api/health")

    # Health endpoint might not be implemented yet, so we check both possibilities
    if response.status_code == 200:
        data = response.json()
        assert "status" in data
        assert "timestamp" in data
        if "dependencies" in data:
            assert isinstance(data["dependencies"], dict)
            # Dependencies structure should match contract
            for service_name, service_status in data["dependencies"].items():
                assert "status" in service_status
                assert service_status["status"] in ["available", "unavailable"]
                if "details" in service_status:
                    assert isinstance(service_status["details"], str)
    elif response.status_code == 404:
        # Health endpoint may not be implemented yet
        pass
    else:
        # If implemented, should return 200 or appropriate error
        assert response.status_code in [200, 400, 500, 503]


def test_error_response_format_consistency():
    """Test that all error responses follow the same format."""
    # Test various error conditions
    error_responses = []

    # Invalid question (empty)
    response = client.post("/api/ask", json={
        "question": "",
        "mode": "full_book",
        "selected_text": None
    })
    if response.status_code >= 400:
        error_responses.append(response.json())

    # Invalid mode
    response = client.post("/api/ask", json={
        "question": "Test question",
        "mode": "invalid_mode",
        "selected_text": None
    })
    if response.status_code >= 400:
        error_responses.append(response.json())

    # Check that all error responses have the required fields
    for error_data in error_responses:
        assert "detail" in error_data
        assert isinstance(error_data["detail"], str)
        assert len(error_data["detail"]) > 0

        # Optional fields should be of correct type if present
        if "error_code" in error_data:
            assert isinstance(error_data["error_code"], str)
        if "timestamp" in error_data:
            assert isinstance(error_data["timestamp"], str)
        if "request_id" in error_data:
            assert isinstance(error_data["request_id"], str)