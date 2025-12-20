from typing import List, Optional
import logging
from ..config.settings import settings

logger = logging.getLogger(__name__)

# Import the Cohere embedding service
from .cohere_embeddings import CohereEmbeddingService

class EmbeddingService:
    """
    Service for generating embeddings using different providers (Cohere, Gemini)
    """

    def __init__(self):
        self.provider = settings.EMBEDDING_PROVIDER.lower()

        if self.provider == "cohere":
            self._cohere_service = CohereEmbeddingService()
            logger.info("Configured for Cohere embedding provider")
        elif self.provider == "gemini":
            import google.generativeai as genai
            self.genai = genai
            if settings.GEMINI_API_KEY:
                self.genai.configure(api_key=settings.GEMINI_API_KEY)
                logger.info("Configured for Gemini embedding provider")
            else:
                logger.warning("GEMINI_API_KEY not set. Gemini embedding provider will use fallback embeddings.")
        else:
            raise ValueError(f"Unsupported embedding provider: {self.provider}")

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for text using the configured provider
        """
        if self.provider == "cohere":
            return await self._cohere_service.generate_embedding(text)
        elif self.provider == "gemini":
            if not settings.GEMINI_API_KEY:
                logger.warning("GEMINI_API_KEY not set, returning default embedding")
                return [0.0] * settings.EMBEDDING_DIMENSION
            try:
                # Use Gemini's embedding functionality
                result = self.genai.embed_content(
                    model=settings.EMBEDDING_MODEL,
                    content=[text],
                    task_type="retrieval_document"
                )
                return result['embedding'][0]
            except Exception as e:
                logger.error(f"Gemini embedding generation failed: {e}")
                return [0.0] * settings.EMBEDDING_DIMENSION
        else:
            raise ValueError(f"Unsupported embedding provider: {self.provider}")

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a batch of texts
        """
        if not texts:
            return []

        if self.provider == "cohere":
            return await self._cohere_service.generate_embeddings_batch(texts)
        elif self.provider == "gemini":
            if not settings.GEMINI_API_KEY:
                logger.warning("GEMINI_API_KEY not set, returning default embeddings for batch")
                return [[0.0] * settings.EMBEDDING_DIMENSION for _ in texts]
            try:
                result = self.genai.embed_content(
                    model=settings.EMBEDDING_MODEL,
                    content=texts,
                    task_type="retrieval_document"
                )
                return result['embedding']
            except Exception as e:
                logger.error(f"Gemini batch embedding generation failed: {e}")
                return [[0.0] * settings.EMBEDDING_DIMENSION for _ in texts]
        else:
            raise ValueError(f"Unsupported embedding provider: {self.provider}")

    def get_embedding_size(self) -> int:
        """
        Get the expected embedding size based on the configured service
        """
        if self.provider == "cohere":
            return self._cohere_service.get_embedding_size()
        else:
            return settings.EMBEDDING_DIMENSION