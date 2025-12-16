"""
Unit tests for RAG (Retrieval-Augmented Generation) model improvements.
These tests verify the enhanced functionality for consistent response quality.
"""
import asyncio
import pytest
from unittest.mock import Mock, patch, AsyncMock

from backend.models.rag import RAGModel, RAGResponse


@pytest.fixture
def rag_model():
    """Create a RAG model instance for testing."""
    with patch('backend.models.rag.EmbeddingService'), \
         patch('qdrant_client.QdrantClient'), \
         patch('google.generativeai.GenerativeModel'):
        model = RAGModel()
        # Mock the Gemini model
        model.generative_model = Mock()
        model.generative_model.generate_content_async = AsyncMock()
        return model


class TestRAGModelEnhancedFunctionality:
    """Test cases for the enhanced RAG model functionality."""

    @pytest.mark.asyncio
    async def test_retrieve_context_from_text_combines_both_sources(self, rag_model):
        """Test that retrieve_context_from_text combines selected text with full database search."""
        query = "What is NVIDIA Isaac Platform?"
        selected_text = "NVIDIA Isaac Platform is a robotics development solution."

        # Mock the embedding service
        rag_model.embedding_service = Mock()
        rag_model.embedding_service.generate_embedding = AsyncMock(return_value=[0.1, 0.2, 0.3])

        # Mock the retrieve_context method to return some database results
        with patch.object(rag_model, 'retrieve_context', new_callable=AsyncMock) as mock_retrieve:
            mock_retrieve.return_value = [
                {
                    "chunk_id": "42",
                    "content": "The NVIDIA Isaac Platform provides tools for robotics development",
                    "score": 0.8,
                    "source": "robotics_handbook_ch3"
                }
            ]

            # Call the method
            result = await rag_model.retrieve_context_from_text(query, selected_text)

            # Verify the result includes both selected text context and database results
            assert len(result) > 0
            # The method should combine both selected text processing and database search
            mock_retrieve.assert_called_once()

    @pytest.mark.asyncio
    async def test_selected_text_mode_with_relevant_text(self, rag_model):
        """Test selected-text mode with relevant text provides meaningful responses."""
        query = "What is NVIDIA Isaac Platform?"
        selected_text = "NVIDIA Isaac Platform is a robotics development solution."

        # Mock embeddings
        rag_model.embedding_service = Mock()
        rag_model.embedding_service.generate_embedding = AsyncMock(return_value=[0.1, 0.2, 0.3])

        # Mock the retrieve_context method to return database results
        with patch.object(rag_model, 'retrieve_context', new_callable=AsyncMock) as mock_retrieve:
            mock_retrieve.return_value = [
                {
                    "chunk_id": "42",
                    "content": "The NVIDIA Isaac Platform provides tools for robotics development",
                    "score": 0.8,
                    "source": "robotics_handbook_ch3"
                }
            ]

            # Mock the generate_response method to return a meaningful response
            with patch.object(rag_model, 'generate_response', new_callable=AsyncMock) as mock_gen_resp:
                mock_gen_resp.return_value = "The NVIDIA Isaac Platform is a comprehensive robotics development solution built on NVIDIA's Omniverse platform."

                # Call process_query with selected_text mode
                result = await rag_model.process_query(query, mode="selected_text", selected_text=selected_text)

                # Verify it returns a meaningful response
                assert isinstance(result, RAGResponse)
                assert result.response  # Response should not be empty
                assert "The book does not provide details about this topic." not in result.response

    @pytest.mark.asyncio
    async def test_selected_text_mode_with_irrelevant_text_fallback(self, rag_model):
        """Test selected-text mode with irrelevant text falls back to full database."""
        query = "What is NVIDIA Isaac Platform?"
        irrelevant_text = "This is completely irrelevant text about something else."

        # Mock embeddings
        rag_model.embedding_service = Mock()
        rag_model.embedding_service.generate_embedding = AsyncMock(return_value=[0.1, 0.2, 0.3])

        # Mock the retrieve_context method to return database results
        with patch.object(rag_model, 'retrieve_context', new_callable=AsyncMock) as mock_retrieve:
            mock_retrieve.return_value = [
                {
                    "chunk_id": "42",
                    "content": "The NVIDIA Isaac Platform provides tools for robotics development",
                    "score": 0.8,
                    "source": "robotics_handbook_ch3"
                }
            ]

            # Mock the generate_response method to return a meaningful response
            with patch.object(rag_model, 'generate_response', new_callable=AsyncMock) as mock_gen_resp:
                mock_gen_resp.return_value = "The NVIDIA Isaac Platform provides tools for robotics development through its comprehensive suite of tools and SDKs."

                # Call process_query with selected_text mode
                result = await rag_model.process_query(query, mode="selected_text", selected_text=irrelevant_text)

                # Verify it returns a meaningful response by falling back to database
                assert isinstance(result, RAGResponse)
                assert result.response  # Response should not be empty
                assert "The book does not provide details about this topic." not in result.response

    @pytest.mark.asyncio
    async def test_full_book_mode_continues_to_work(self, rag_model):
        """Test that full-book mode continues to work correctly."""
        query = "What is NVIDIA Isaac Platform?"

        # Mock embeddings
        rag_model.embedding_service = Mock()
        rag_model.embedding_service.generate_embedding = AsyncMock(return_value=[0.1, 0.2, 0.3])

        # Mock the retrieve_context method to return database results
        with patch.object(rag_model, 'retrieve_context', new_callable=AsyncMock) as mock_retrieve:
            mock_retrieve.return_value = [
                {
                    "chunk_id": "42",
                    "content": "The NVIDIA Isaac Platform provides tools for robotics development",
                    "score": 0.8,
                    "source": "robotics_handbook_ch3"
                }
            ]

            # Mock the generate_response method
            with patch.object(rag_model, 'generate_response', new_callable=AsyncMock) as mock_gen_resp:
                mock_gen_resp.return_value = "The NVIDIA Isaac Platform is a comprehensive robotics development solution."

                # Call process_query with full_book mode
                result = await rag_model.process_query(query, mode="full_book")

                # Verify it returns a meaningful response
                assert isinstance(result, RAGResponse)
                assert result.response == "The NVIDIA Isaac Platform is a comprehensive robotics development solution."

    @pytest.mark.asyncio
    async def test_consistent_response_quality_across_modes(self, rag_model):
        """Test that both modes return consistent response quality."""
        query = "What is NVIDIA Isaac Platform?"
        selected_text = "NVIDIA Isaac Platform is a robotics development solution."

        # Mock embeddings
        rag_model.embedding_service = Mock()
        rag_model.embedding_service.generate_embedding = AsyncMock(return_value=[0.1, 0.2, 0.3])

        # Mock the retrieve_context method to return database results
        with patch.object(rag_model, 'retrieve_context', new_callable=AsyncMock) as mock_retrieve:
            mock_retrieve.return_value = [
                {
                    "chunk_id": "42",
                    "content": "The NVIDIA Isaac Platform provides tools for robotics development",
                    "score": 0.8,
                    "source": "robotics_handbook_ch3"
                }
            ]

            # Mock the generate_response method
            with patch.object(rag_model, 'generate_response', new_callable=AsyncMock) as mock_gen_resp:
                mock_gen_resp.return_value = "The NVIDIA Isaac Platform is a comprehensive robotics development solution."

                # Get response from selected_text mode
                result_selected = await rag_model.process_query(query, mode="selected_text", selected_text=selected_text)

                # Get response from full_book mode
                result_full = await rag_model.process_query(query, mode="full_book")

                # Both should return meaningful responses (not the fallback message)
                assert "The book does not provide details about this topic." not in result_selected.response
                assert "The book does not provide details about this topic." not in result_full.response

    @pytest.mark.asyncio
    async def test_extremely_short_selected_text_handling(self, rag_model):
        """Test handling of extremely short selected text."""
        query = "What is NVIDIA Isaac Platform?"
        short_text = "Hi"  # Very short text

        # Mock embeddings
        rag_model.embedding_service = Mock()
        rag_model.embedding_service.generate_embedding = AsyncMock(return_value=[0.1, 0.2, 0.3])

        # Mock the retrieve_context method to return database results
        with patch.object(rag_model, 'retrieve_context', new_callable=AsyncMock) as mock_retrieve:
            mock_retrieve.return_value = [
                {
                    "chunk_id": "42",
                    "content": "The NVIDIA Isaac Platform provides tools for robotics development",
                    "score": 0.8,
                    "source": "robotics_handbook_ch3"
                }
            ]

            # Mock the generate_response method to return a meaningful response
            with patch.object(rag_model, 'generate_response', new_callable=AsyncMock) as mock_gen_resp:
                mock_gen_resp.return_value = "The NVIDIA Isaac Platform is an end-to-end robotics development solution built on NVIDIA's Omniverse platform."

                # Call process_query with selected_text mode
                result = await rag_model.process_query(query, mode="selected_text", selected_text=short_text)

                # Should still return meaningful response by falling back to database
                assert isinstance(result, RAGResponse)
                assert result.response  # Response should not be empty
                assert "The book does not provide details about this topic." not in result.response

    @pytest.mark.asyncio
    async def test_special_characters_selected_text_handling(self, rag_model):
        """Test handling of selected text with only special characters."""
        query = "What is NVIDIA Isaac Platform?"
        special_text = "!@#$%^&*()"  # Only special characters

        # Mock embeddings
        rag_model.embedding_service = Mock()
        rag_model.embedding_service.generate_embedding = AsyncMock(return_value=[0.1, 0.2, 0.3])

        # Mock the retrieve_context method to return database results
        with patch.object(rag_model, 'retrieve_context', new_callable=AsyncMock) as mock_retrieve:
            mock_retrieve.return_value = [
                {
                    "chunk_id": "42",
                    "content": "The NVIDIA Isaac Platform provides tools for robotics development",
                    "score": 0.8,
                    "source": "robotics_handbook_ch3"
                }
            ]

            # Mock the generate_response method to return a meaningful response
            with patch.object(rag_model, 'generate_response', new_callable=AsyncMock) as mock_gen_resp:
                mock_gen_resp.return_value = "The NVIDIA Isaac Platform provides comprehensive tools for robotics development, built on NVIDIA's Omniverse platform."

                # Call process_query with selected_text mode
                result = await rag_model.process_query(query, mode="selected_text", selected_text=special_text)

                # Should still return meaningful response by falling back to database
                assert isinstance(result, RAGResponse)
                assert result.response  # Response should not be empty
                assert "The book does not provide details about this topic." not in result.response


class TestRAGResponseEnhancements:
    """Test cases for enhanced RAGResponse functionality."""

    def test_rag_response_structure(self):
        """Test the structure of RAGResponse."""
        response_text = "Test response"
        sources = [
            {
                "chunk_id": "42",
                "content": "Test content",
                "score": 0.8,
                "source": "test_source"
            }
        ]
        references = ["test_source"]

        rag_response = RAGResponse(
            response=response_text,
            sources=sources,
            references=references
        )

        assert rag_response.response == response_text
        assert len(rag_response.sources) == 1
        assert rag_response.sources[0]["chunk_id"] == "42"
        assert rag_response.sources[0]["content"] == "Test content"
        assert rag_response.sources[0]["score"] == 0.8
        assert rag_response.sources[0]["source"] == "test_source"
        assert rag_response.references == ["test_source"]