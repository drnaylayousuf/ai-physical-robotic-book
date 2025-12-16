import asyncio
import logging
from datetime import datetime, timedelta
from typing import Dict, Optional

import httpx
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.exceptions import UnexpectedResponse

from src.config.settings import settings

# Configure logging
logger = logging.getLogger(__name__)


class DependencyChecker:
    """
    Service class for checking the availability of external dependencies
    like vector database and embedding services.
    """

    def __init__(self):
        self._qdrant_client: Optional[QdrantClient] = None
        self._qdrant_last_check: Optional[datetime] = None
        self._qdrant_status: bool = False

        self._embedding_last_check: Optional[datetime] = None
        self._embedding_status: bool = False

    async def check_vector_database(self) -> bool:
        """
        Check if the vector database (Qdrant) is available.

        Returns:
            True if available, False otherwise
        """
        # Check if we have a recent cached result
        now = datetime.utcnow()
        if (self._qdrant_last_check and
            now - self._qdrant_last_check < timedelta(seconds=settings.dependency_validation_cache_ttl)):
            return self._qdrant_status

        try:
            # Initialize Qdrant client if not already done
            if self._qdrant_client is None:
                self._qdrant_client = QdrantClient(
                    url=settings.qdrant_url,
                    api_key=settings.qdrant_api_key,
                    timeout=5.0  # 5 second timeout
                )

            # Test connection by getting cluster status
            cluster_info = self._qdrant_client.cluster_status()

            # Test collection existence
            try:
                collection_info = self._qdrant_client.get_collection(settings.qdrant_collection_name)
                logger.info(f"Qdrant collection '{settings.qdrant_collection_name}' exists with {collection_info.points_count} points")
            except UnexpectedResponse:
                logger.warning(f"Qdrant collection '{settings.qdrant_collection_name}' does not exist")
                # Collection not existing is not necessarily an error, just means it's empty
                pass

            self._qdrant_status = True
            self._qdrant_last_check = now
            logger.info("Vector database (Qdrant) is available")
            return True

        except Exception as e:
            logger.error(f"Vector database (Qdrant) check failed: {str(e)}")
            self._qdrant_status = False
            self._qdrant_last_check = now
            return False

    async def check_embedding_service(self) -> bool:
        """
        Check if the embedding service (Cohere) is available.
        This validates that the API key is configured and the service is accessible.

        Returns:
            True if available, False otherwise
        """
        # Check if we have a recent cached result
        now = datetime.utcnow()
        if (self._embedding_last_check and
            now - self._embedding_last_check < timedelta(seconds=settings.dependency_validation_cache_ttl)):
            return self._embedding_status

        try:
            # Check if API key is provided
            if not settings.cohere_api_key:
                logger.error("Cohere API key not configured")
                self._embedding_status = False
                self._embedding_last_check = now
                return False

            # Create an embedding service instance to test the connection
            from src.services.embedding_service import EmbeddingService
            embedding_service = EmbeddingService()

            # Validate the connection by making a test call
            is_valid = await self._validate_cohere_connection(embedding_service)

            if is_valid:
                logger.info("Embedding service (Cohere) is available and accessible")
                self._embedding_status = True
                self._embedding_last_check = now
                return True
            else:
                logger.error("Embedding service (Cohere) is not accessible")
                self._embedding_status = False
                self._embedding_last_check = now
                return False

        except Exception as e:
            logger.error(f"Embedding service (Cohere) check failed: {str(e)}")
            self._embedding_status = False
            self._embedding_last_check = now
            return False

    async def _validate_cohere_connection(self, embedding_service) -> bool:
        """
        Validate the Cohere connection by making a test call.

        Args:
            embedding_service: The embedding service instance to validate

        Returns:
            True if connection is valid, False otherwise
        """
        return await embedding_service.validate_connection()

    async def get_dependency_status(self) -> Dict[str, Dict[str, str]]:
        """
        Get the status of all dependencies.

        Returns:
            Dictionary with dependency status information
        """
        vector_db_available = await self.check_vector_database()
        embedding_available = await self.check_embedding_service()

        return {
            "vector_database": {
                "status": "available" if vector_db_available else "unavailable",
                "last_checked": self._qdrant_last_check.isoformat() + "Z" if self._qdrant_last_check else None,
                "details": "Qdrant vector database connection successful" if vector_db_available else "Failed to connect to Qdrant"
            },
            "embedding_service": {
                "status": "available" if embedding_available else "unavailable",
                "last_checked": self._embedding_last_check.isoformat() + "Z" if self._embedding_last_check else None,
                "details": "Cohere embedding service configured" if embedding_available else "Cohere API key not configured or invalid"
            }
        }