"""
Integration tests for embedding service validation.
"""
import pytest
from unittest.mock import patch, MagicMock
from src.services.embedding_service import EmbeddingService
from src.services.dependency_checker import DependencyChecker
from src.api.middleware.error_handler import EmbeddingServiceException
from src.config.settings import settings


@pytest.mark.asyncio
async def test_dependency_checker_embedding_service_available():
    """Test that dependency checker correctly identifies when embedding service is available."""
    # Temporarily set a mock API key for testing
    original_key = settings.cohere_api_key
    settings.cohere_api_key = "test-key"

    try:
        checker = DependencyChecker()

        # Mock the validation method to return True
        with patch.object(checker, '_validate_cohere_connection', return_value=True):
            result = await checker.check_embedding_service()
            assert result is True
    finally:
        # Restore original value
        settings.cohere_api_key = original_key


@pytest.mark.asyncio
async def test_dependency_checker_embedding_service_unavailable():
    """Test that dependency checker correctly identifies when embedding service is unavailable."""
    # Temporarily clear the API key
    original_key = settings.cohere_api_key
    settings.cohere_api_key = ""

    try:
        checker = DependencyChecker()
        result = await checker.check_embedding_service()
        assert result is False
    finally:
        # Restore original value
        settings.cohere_api_key = original_key


@pytest.mark.asyncio
async def test_embedding_service_with_mocked_cohere():
    """Test embedding service with mocked Cohere API calls."""
    # Temporarily set a mock API key for testing
    original_key = settings.cohere_api_key
    settings.cohere_api_key = "test-key"

    try:
        service = EmbeddingService()

        # Mock the Cohere client
        with patch('cohere.Client') as mock_cohere:
            mock_client = MagicMock()
            mock_cohere.return_value = mock_client

            # Mock the embed method to return test embeddings
            mock_response = MagicMock()
            mock_response.embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
            mock_client.embed.return_value = mock_response

            # Test generating embeddings
            texts = ["test text 1", "test text 2"]
            result = await service.generate_embeddings(texts)

            assert len(result) == 2
            assert len(result[0]) == 3  # 3 dimensions in mock
            assert len(result[1]) == 3  # 3 dimensions in mock

            # Verify the client was called with correct parameters
            mock_client.embed.assert_called_once_with(
                texts=texts,
                model="embed-english-v3.0",
                input_type="search_document"
            )
    finally:
        # Restore original value
        settings.cohere_api_key = original_key


@pytest.mark.asyncio
async def test_embedding_service_query_embedding():
    """Test generating a single query embedding."""
    # Temporarily set a mock API key for testing
    original_key = settings.cohere_api_key
    settings.cohere_api_key = "test-key"

    try:
        service = EmbeddingService()

        # Mock the Cohere client
        with patch('cohere.Client') as mock_cohere:
            mock_client = MagicMock()
            mock_cohere.return_value = mock_client

            # Mock the embed method to return test embeddings
            mock_response = MagicMock()
            mock_response.embeddings = [[0.1, 0.2, 0.3]]
            mock_client.embed.return_value = mock_response

            # Test generating query embedding
            query = "test query"
            result = await service.generate_query_embedding(query)

            assert len(result) == 3  # 3 dimensions in mock

            # Verify the client was called with correct parameters
            mock_client.embed.assert_called_once_with(
                texts=[query],
                model="embed-english-v3.0",
                input_type="search_document"
            )
    finally:
        # Restore original value
        settings.cohere_api_key = original_key


@pytest.mark.asyncio
async def test_dependency_checker_get_status():
    """Test getting comprehensive dependency status."""
    # Temporarily set mock API key
    original_key = settings.cohere_api_key
    settings.cohere_api_key = "test-key"

    try:
        checker = DependencyChecker()

        # Mock both checks to return True
        with patch.object(checker, 'check_vector_database', return_value=True), \
             patch.object(checker, 'check_embedding_service', return_value=True):

            status = await checker.get_dependency_status()

            assert "vector_database" in status
            assert "embedding_service" in status
            assert status["vector_database"]["status"] == "available"
            assert status["embedding_service"]["status"] == "available"
    finally:
        # Restore original value
        settings.cohere_api_key = original_key


@pytest.mark.asyncio
async def test_embedding_service_error_handling():
    """Test error handling in embedding service."""
    service = EmbeddingService()

    # Mock the client to raise an exception
    with patch.object(service, '_get_client') as mock_get_client:
        mock_get_client.side_effect = Exception("API Error")

        with pytest.raises(EmbeddingServiceException):
            await service.generate_embeddings(["test text"])