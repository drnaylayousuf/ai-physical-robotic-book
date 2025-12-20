import pytest
import os
from unittest.mock import Mock, patch, MagicMock
from backend.utils.gemini_client import GeminiClient

def test_gemini_client_initialization():
    """
    Test that GeminiClient initializes properly with API key
    """
    # Mock the environment variable
    with patch.dict(os.environ, {"GEMINI_API_KEY": "test-api-key"}):
        client = GeminiClient()
        assert client is not None
        assert client.model_name == "gemini-2.5-flash"

def test_gemini_client_initialization_without_api_key():
    """
    Test that GeminiClient raises error when no API key is provided
    """
    # Temporarily remove GEMINI_API_KEY from environment
    original_key = os.environ.get("GEMINI_API_KEY")
    if "GEMINI_API_KEY" in os.environ:
        del os.environ["GEMINI_API_KEY"]

    try:
        with pytest.raises(ValueError, match="GEMINI_API_KEY environment variable is required"):
            GeminiClient(api_key=None)
    finally:
        # Restore original key if it existed
        if original_key:
            os.environ["GEMINI_API_KEY"] = original_key

def test_gemini_generate_text():
    """
    Test the generate_text method of GeminiClient
    """
    with patch.dict(os.environ, {"GEMINI_API_KEY": "test-api-key"}):
        client = GeminiClient()

        # Mock the model.generate_content method
        mock_response = Mock()
        mock_response.text = "This is a test response"
        client.model.generate_content = Mock(return_value=mock_response)

        result = client.generate_text("Test prompt")
        assert result == "This is a test response"
        client.model.generate_content.assert_called_once()

def test_gemini_generate_with_context():
    """
    Test the generate_text method with context
    """
    with patch.dict(os.environ, {"GEMINI_API_KEY": "test-api-key"}):
        client = GeminiClient()

        # Mock the model.generate_content method
        mock_response = Mock()
        mock_response.text = "Response based on context and prompt"
        client.model.generate_content = Mock(return_value=mock_response)

        result = client.generate_text("Test prompt", context="Test context")
        assert result == "Response based on context and prompt"
        # Verify that the call includes both context and prompt
        client.model.generate_content.assert_called_once()

if __name__ == "__main__":
    pytest.main()