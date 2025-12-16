from typing import List, Optional
import logging
from datetime import datetime

from src.models.request import RAGRequest, RAGMode
from src.models.response import RAGResponse, RetrievedChunk
from src.models.chunk import RetrievedChunk as ModelRetrievedChunk
from src.services.embedding_service import EmbeddingService
from src.services.dependency_checker import DependencyChecker
from src.api.middleware.error_handler import (
    RAGException,
    VectorDatabaseException,
    EmbeddingServiceException,
    InvalidRequestException
)
from src.config.settings import settings

# Configure logging
logger = logging.getLogger(__name__)


class RAGService:
    """
    Service class for handling RAG (Retrieval-Augmented Generation) operations.
    """

    def __init__(self):
        self.embedding_service = EmbeddingService()
        self.dependency_checker = DependencyChecker()

    async def process_query(self, request: RAGRequest) -> RAGResponse:
        """
        Process a RAG query based on the request parameters.

        Args:
            request: The RAG request containing question, mode, and optional selected_text

        Returns:
            RAGResponse containing the answer and sources
        """
        try:
            logger.info(f"Processing query: {request.question[:50]}... | Mode: {request.mode}")

            # Handle different modes
            if request.mode == RAGMode.FULL_BOOK:
                return await self._process_full_book_mode(request)
            elif request.mode == RAGMode.SELECTED_TEXT:
                return await self._process_selected_text_mode(request)
            else:
                raise InvalidRequestException(f"Unsupported mode: {request.mode}")

        except RAGException:
            # Re-raise RAG-specific exceptions
            raise
        except Exception as e:
            logger.error(f"Error processing query: {str(e)}")
            raise RAGException(f"Error processing your question: {str(e)}")

    async def _process_full_book_mode(self, request: RAGRequest) -> RAGResponse:
        """
        Process query in full book mode - search entire collection for relevant chunks.
        """
        try:
            # Verify vector database is available
            if not await self.dependency_checker.check_vector_database():
                raise VectorDatabaseException(
                    "Vector database is not available. Please check if Qdrant is running and accessible."
                )

            # For now, return a placeholder response until we implement the full RAG pipeline
            # In a real implementation, this would:
            # 1. Generate embeddings for the question
            # 2. Search the vector database for relevant chunks
            # 3. Use an LLM to generate a response based on the context
            # 4. Return the response with sources

            # For now, return a descriptive response that explains the process
            response_text = f"Based on the provided context, here is the answer to: '{request.question}'"

            # This is where the actual RAG logic would go
            # For now, we'll return an empty sources list and references
            sources = []
            references = []

            return RAGResponse(
                response=response_text,
                sources=sources,
                references=references
            )

        except VectorDatabaseException:
            raise
        except Exception as e:
            logger.error(f"Error in full book mode: {str(e)}")
            raise VectorDatabaseException(f"Error processing full book query: {str(e)}")

    async def _process_selected_text_mode(self, request: RAGRequest) -> RAGResponse:
        """
        Process query in selected text mode - work with provided text.
        """
        try:
            # Validate that selected_text is provided for this mode
            if request.selected_text is None:
                raise InvalidRequestException(
                    "selected_text parameter is required for 'selected_text' mode"
                )

            # Verify embedding service is available
            if not await self.dependency_checker.check_embedding_service():
                raise EmbeddingServiceException(
                    "Embedding service is not available. Please check Cohere API access."
                )

            # For now, return a placeholder response until we implement the full pipeline
            # In a real implementation, this would:
            # 1. Clean and chunk the provided text
            # 2. Generate embeddings for the chunks
            # 3. Find relevant parts based on the question
            # 4. Use an LLM to generate a response based on the context

            response_text = f"Based on the provided text, here is the answer to: '{request.question}'"

            # This is where the actual RAG logic would go
            sources = []
            references = []

            return RAGResponse(
                response=response_text,
                sources=sources,
                references=references
            )

        except EmbeddingServiceException:
            raise
        except Exception as e:
            logger.error(f"Error in selected text mode: {str(e)}")
            raise EmbeddingServiceException(f"Error processing selected text query: {str(e)}")

    async def validate_request(self, request: RAGRequest) -> bool:
        """
        Validate the RAG request parameters.

        Args:
            request: The RAG request to validate

        Returns:
            True if valid, raises exception if invalid
        """
        if not request.question or not request.question.strip():
            raise InvalidRequestException("Question cannot be empty or contain only whitespace")

        if request.mode not in [RAGMode.FULL_BOOK, RAGMode.SELECTED_TEXT]:
            raise InvalidRequestException(f"Invalid mode: {request.mode}. Must be 'full_book' or 'selected_text'")

        if request.mode == RAGMode.SELECTED_TEXT and request.selected_text is None:
            # selected_text can be null for selected_text mode, but if provided it should be valid
            pass

        return True