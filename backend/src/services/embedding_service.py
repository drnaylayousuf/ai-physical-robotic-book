import asyncio
import logging
from typing import List, Optional

import cohere
from cohere import EmbedResponse

from src.config.settings import settings
from src.api.middleware.error_handler import EmbeddingServiceException

# Configure logging
logger = logging.getLogger(__name__)


class EmbeddingService:
    """
    Service class for handling embedding generation using Cohere API.
    """

    def __init__(self):
        self._client: Optional[cohere.Client] = None
        self._initialized = False

    def _get_client(self) -> cohere.Client:
        """
        Get or initialize the Cohere client.

        Returns:
            Cohere client instance
        """
        if not self._initialized or self._client is None:
            if not settings.cohere_api_key:
                raise EmbeddingServiceException("Cohere API key is not configured")

            try:
                self._client = cohere.Client(api_key=settings.cohere_api_key)
                self._initialized = True
                logger.info("Cohere client initialized successfully")
            except Exception as e:
                logger.error(f"Failed to initialize Cohere client: {str(e)}")
                raise EmbeddingServiceException(f"Failed to initialize Cohere client: {str(e)}")

        return self._client

    async def generate_embeddings(self, texts: List[str], model: str = "embed-english-v3.0") -> List[List[float]]:
        """
        Generate embeddings for the provided texts.

        Args:
            texts: List of text strings to generate embeddings for
            model: The embedding model to use

        Returns:
            List of embedding vectors (each vector is a list of floats)
        """
        try:
            # Cohere client is synchronous, so we run it in a thread pool
            loop = asyncio.get_event_loop()
            response: EmbedResponse = await loop.run_in_executor(
                None,
                lambda: self._get_client().embed(
                    texts=texts,
                    model=model,
                    input_type="search_document"  # or "search_query" for queries
                )
            )

            if not response or not response.embeddings:
                raise EmbeddingServiceException("No embeddings returned from Cohere API")

            logger.info(f"Successfully generated embeddings for {len(texts)} text(s)")
            return response.embeddings

        except Exception as e:
            logger.error(f"Error generating embeddings: {str(e)}")
            raise EmbeddingServiceException(f"Error generating embeddings: {str(e)}")

    async def generate_query_embedding(self, query: str, model: str = "embed-english-v3.0") -> List[float]:
        """
        Generate embedding for a single query text.

        Args:
            query: The query text to generate embedding for
            model: The embedding model to use

        Returns:
            Embedding vector (list of floats)
        """
        try:
            embeddings = await self.generate_embeddings([query], model)
            if embeddings and len(embeddings) > 0:
                return embeddings[0]
            else:
                raise EmbeddingServiceException("No embedding returned for query")
        except Exception as e:
            logger.error(f"Error generating query embedding: {str(e)}")
            raise EmbeddingServiceException(f"Error generating query embedding: {str(e)}")

    async def validate_connection(self) -> bool:
        """
        Validate that the embedding service is accessible.

        Returns:
            True if connection is successful, False otherwise
        """
        try:
            # Test with a simple embedding request
            test_embedding = await self.generate_query_embedding("test connection")
            return test_embedding is not None and len(test_embedding) > 0
        except Exception as e:
            logger.error(f"Embedding service connection validation failed: {str(e)}")
            return False