"""
Unit tests for the embedding service.
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
from src.services.embedding_service import EmbeddingService
from src.api.middleware.error_handler import EmbeddingServiceException
from src.config.settings import settings


@pytest.fixture
def embedding_service():
    """Create an embedding service instance for testing."""
    # Temporarily set a mock API key for testing
    original_key = settings.cohere_api_key
    settings.cohere_api_key = "test-key"

    service = EmbeddingService()
    yield service

    # Restore original value
    settings.cohere_api_key = original_key


@pytest.mark.asyncio
async def test_embedding_service_initialization():
    """Test that the embedding service initializes correctly."""
    service = EmbeddingService()

    # Should not raise an exception during initialization
    assert service is not None
    assert service._client is None  # Client not created until needed


@pytest.mark.asyncio
async def test_embedding_service_initialization_no_api_key():
    """Test that the embedding service raises an error when no API key is provided."""
    # Temporarily clear the API key
    original_key = settings.cohere_api_key
    settings.cohere_api_key = ""

    try:
        service = EmbeddingService()

        # This should raise an exception when we try to use the service
        with pytest.raises(EmbeddingServiceException):
            await service.generate_query_embedding("test")
    finally:
        # Restore original value
        settings.cohere_api_key = original_key


@pytest.mark.asyncio
@patch('cohere.Client')
async def test_generate_embeddings_success(mock_cohere_client):
    """Test that embeddings can be generated successfully."""
    # Set up mock
    mock_client_instance = Mock()
    mock_cohere_client.return_value = mock_client_instance
    mock_client_instance.embed.return_value = Mock()
    mock_client_instance.embed.return_value.embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]

    service = EmbeddingService()
    service._client = mock_client_instance
    service._initialized = True

    texts = ["test text 1", "test text 2"]
    result = await service.generate_embeddings(texts)

    assert len(result) == 2
    assert len(result[0]) == 3  # 3 dimensions in mock
    assert len(result[1]) == 3  # 3 dimensions in mock
    mock_client_instance.embed.assert_called_once_with(
        texts=texts,
        model="embed-english-v3.0",
        input_type="search_document"
    )


@pytest.mark.asyncio
@patch('cohere.Client')
async def test_generate_query_embedding_success(mock_cohere_client):
    """Test that a single query embedding can be generated successfully."""
    # Set up mock
    mock_client_instance = Mock()
    mock_cohere_client.return_value = mock_client_instance
    mock_client_instance.embed.return_value = Mock()
    mock_client_instance.embed.return_value.embeddings = [[0.1, 0.2, 0.3]]

    service = EmbeddingService()
    service._client = mock_client_instance
    service._initialized = True

    query = "test query"
    result = await service.generate_query_embedding(query)

    assert len(result) == 3  # 3 dimensions in mock
    mock_client_instance.embed.assert_called_once_with(
        texts=[query],
        model="embed-english-v3.0",
        input_type="search_document"
    )


@pytest.mark.asyncio
async def test_generate_embeddings_error_handling():
    """Test that errors during embedding generation are properly handled."""
    service = EmbeddingService()

    # Mock the client to raise an exception
    with patch.object(service, '_get_client') as mock_get_client:
        mock_get_client.side_effect = Exception("API Error")

        with pytest.raises(EmbeddingServiceException):
            await service.generate_embeddings(["test text"])


@pytest.mark.asyncio
async def test_generate_query_embedding_error_handling():
    """Test that errors during query embedding generation are properly handled."""
    service = EmbeddingService()

    # Mock the client to raise an exception
    with patch.object(service, '_get_client') as mock_get_client:
        mock_get_client.side_effect = Exception("API Error")

        with pytest.raises(EmbeddingServiceException):
            await service.generate_query_embedding("test query")


@pytest.mark.asyncio
async def test_validate_connection_success():
    """Test that connection validation works when successful."""
    service = EmbeddingService()

    # Mock successful embedding generation
    with patch.object(service, 'generate_query_embedding') as mock_gen:
        mock_gen.return_value = [0.1, 0.2, 0.3]

        result = await service.validate_connection()
        assert result is True


@pytest.mark.asyncio
async def test_validate_connection_failure():
    """Test that connection validation fails when embedding generation fails."""
    service = EmbeddingService()

    # Mock failed embedding generation
    with patch.object(service, 'generate_query_embedding') as mock_gen:
        mock_gen.side_effect = EmbeddingServiceException("Connection failed")

        result = await service.validate_connection()
        assert result is False