from typing import List, Optional
import logging
import cohere
from ..config.settings import settings

logger = logging.getLogger(__name__)

class CohereEmbeddingService:
    """
    Service for generating embeddings using Cohere API
    """

    def __init__(self):
        if not settings.COHERE_API_KEY:
            logger.warning("COHERE_API_KEY not set. Cohere embedding service will use fallback embeddings.")
            self.client = None
            self.model = settings.EMBEDDING_MODEL if settings.EMBEDDING_MODEL != "text-embedding-3-large" else "embed-english-v3.0"
        else:
            self.client = cohere.Client(settings.COHERE_API_KEY)
            self.model = settings.EMBEDDING_MODEL if settings.EMBEDDING_MODEL != "text-embedding-3-large" else "embed-english-v3.0"
            logger.info(f"Configured Cohere embedding service with model: {self.model}")

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for text using Cohere API
        """
        if self.client is None:
            # Return a default embedding as fallback when API key is not set
            logger.warning("Cohere client not available, returning default embedding")
            return [0.0] * settings.EMBEDDING_DIMENSION

        try:
            # Use the Cohere API to generate embeddings
            response = self.client.embed(
                texts=[text],
                model=self.model,
                input_type="search_document"  # Using search_document for knowledge base content
            )

            # Extract the embedding from the response
            embedding = response.embeddings[0]
            return embedding
        except Exception as e:
            logger.error(f"Cohere embedding generation failed: {e}")
            # Return a default embedding as fallback with correct dimensions
            return [0.0] * settings.EMBEDDING_DIMENSION

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a batch of texts
        """
        if not texts:
            return []

        if self.client is None:
            # Return default embeddings as fallback when API key is not set
            logger.warning("Cohere client not available, returning default embeddings for batch")
            return [[0.0] * settings.EMBEDDING_DIMENSION for _ in texts]

        try:
            # Use the Cohere API to generate embeddings for batch
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type="search_document"  # Using search_document for knowledge base content
            )

            # Extract embeddings from the response
            embeddings = response.embeddings
            return embeddings
        except Exception as e:
            logger.error(f"Cohere batch embedding generation failed: {e}")
            # Return default embeddings as fallback
            return [[0.0] * settings.EMBEDDING_DIMENSION for _ in texts]

    def get_embedding_size(self) -> int:
        """
        Get the expected embedding size based on the configured Cohere model
        """
        # Different Cohere models have different dimensions
        model = self.model.lower()
        if "embed-multilingual" in model:
            return 1024  # Multilingual models typically have 1024 dimensions
        elif "embed-english-v3" in model:
            return 1024  # English v3 models have 1024 dimensions
        elif "embed-english-light-v3" in model:
            return 384   # Light models have 384 dimensions
        else:
            # Default to the setting if model is unknown
            return settings.EMBEDDING_DIMENSION