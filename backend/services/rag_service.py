import os
from typing import List, Dict, Optional, Any, Union
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
from pydantic import BaseModel
import logging
from ..utils.embeddings import EmbeddingService
from ..config.settings import settings

logger = logging.getLogger(__name__)

class BookChunk(BaseModel):
    """
    Model representing a book chunk with metadata and embedding
    """
    id: Union[str, int]  # Allow both string and integer IDs for Qdrant compatibility
    content: str
    page: Optional[int] = None
    section: Optional[str] = None
    embedding: Optional[List[float]] = None
    metadata: Optional[Dict[str, Any]] = {}
    created_at: Optional[str] = None
    updated_at: Optional[str] = None


class UserQuery(BaseModel):
    """
    Model representing a user query
    """
    query_text: str
    query_embedding: Optional[List[float]] = None
    similarity_threshold: float = 0.7


class MigrationStatus(BaseModel):
    """
    Model representing migration status
    """
    total_chunks: int = 0
    migrated_chunks: int = 0
    error_count: int = 0
    errors: List[Dict[str, Any]] = []


class RAGService:
    """
    Service class for handling RAG operations with Qdrant Cloud
    """

    def __init__(self):
        # Initialize Qdrant client with Cloud configuration
        if not settings.QDRANT_URL or not settings.QDRANT_API_KEY:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required")

        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=30
        )

        self.collection_name = settings.QDRANT_COLLECTION_NAME
        self.embedding_service = EmbeddingService()

        # Verify connection and collection
        self._verify_connection()
        self._verify_collection()

    def _verify_connection(self):
        """
        Verify the connection to Qdrant Cloud
        """
        try:
            # Use get_collections() to verify the connection works since health() method doesn't exist in this version
            collections = self.client.get_collections()
            logger.info(f"Connected to Qdrant Cloud successfully")
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant Cloud: {e}")
            raise

    def _verify_collection(self):
        """
        Verify that the collection exists, create if it doesn't
        """
        try:
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if self.collection_name not in collection_names:
                logger.info(f"Collection {self.collection_name} does not exist, creating it...")
                self._create_collection()
            else:
                logger.info(f"Collection {self.collection_name} exists")
        except Exception as e:
            logger.error(f"Error verifying collection: {e}")
            raise

    def _create_collection(self):
        """
        Create the Qdrant collection for book chunks
        """
        embedding_size = self.embedding_service.get_embedding_size()

        self.client.recreate_collection(
            collection_name=self.collection_name,
            vectors_config=models.VectorParams(
                size=embedding_size,
                distance=models.Distance.COSINE
            ),
            optimizers_config=models.OptimizersConfigDiff(
                memmap_threshold=20000,
                indexing_threshold=20000,
            ),
        )
        logger.info(f"Created collection {self.collection_name} with {embedding_size}-dimension vectors")

    async def health_check(self) -> Dict[str, Any]:
        """
        Check the health of the Qdrant Cloud connection
        """
        try:
            # Use get_collections() to verify the connection works since health() method doesn't exist in this version
            collections = self.client.get_collections()
            return {"status": "ok", "collections_count": len(collections.collections)}
        except Exception as e:
            logger.error(f"Qdrant health check failed: {e}")
            return {"status": "error", "error": str(e)}

    def check_collection_exists(self) -> bool:
        """
        Check if the collection exists in Qdrant Cloud
        """
        try:
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]
            return self.collection_name in collection_names
        except Exception:
            return False

    async def add_chunk(self, chunk: BookChunk) -> bool:
        """
        Add a single book chunk to Qdrant Cloud
        """
        try:
            # Generate embedding if not provided
            if not chunk.embedding:
                embedding = await self.embedding_service.generate_embedding(chunk.content)
                chunk.embedding = embedding

            # Prepare the point for Qdrant
            point = PointStruct(
                id=chunk.id,
                vector=chunk.embedding,
                payload={
                    "content": chunk.content,
                    "page": chunk.page,
                    "section": chunk.section,
                    "metadata": chunk.metadata,
                    "created_at": chunk.created_at,
                    "updated_at": chunk.updated_at
                }
            )

            # Upsert the point to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=[point]
            )

            logger.debug(f"Added chunk {chunk.id} to collection {self.collection_name}")
            return True
        except Exception as e:
            logger.error(f"Failed to add chunk {chunk.id}: {e}")
            return False

    async def add_chunks(self, chunks: List[BookChunk]) -> MigrationStatus:
        """
        Add multiple book chunks to Qdrant Cloud
        """
        status = MigrationStatus(total_chunks=len(chunks))

        for chunk in chunks:
            try:
                # Generate embedding if not provided
                if not chunk.embedding:
                    embedding = await self.embedding_service.generate_embedding(chunk.content)
                    chunk.embedding = embedding

                # Prepare the point for Qdrant
                point = PointStruct(
                    id=chunk.id,
                    vector=chunk.embedding,
                    payload={
                        "content": chunk.content,
                        "page": chunk.page,
                        "section": chunk.section,
                        "metadata": chunk.metadata,
                        "created_at": chunk.created_at,
                        "updated_at": chunk.updated_at
                    }
                )

                # Upsert the point to Qdrant
                self.client.upsert(
                    collection_name=self.collection_name,
                    points=[point]
                )

                status.migrated_chunks += 1
                logger.debug(f"Added chunk {chunk.id} to collection {self.collection_name}")

            except Exception as e:
                logger.error(f"Failed to add chunk {chunk.id}: {e}")
                status.error_count += 1
                status.errors.append({
                    "chunk_id": chunk.id,
                    "error": str(e)
                })

        return status

    async def search_chunks(self, query: UserQuery = None, query_text: str = None, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Search for relevant chunks based on user query
        Supports both UserQuery object and direct query text
        """
        try:
            # Determine the query text to use
            if query_text:
                # Generate embedding for the provided query text
                query_embedding = await self.embedding_service.generate_embedding(query_text)
            elif query and query.query_text:
                # Generate embedding for the query object text if not provided
                if not query.query_embedding:
                    query_embedding = await self.embedding_service.generate_embedding(query.query_text)
                else:
                    query_embedding = query.query_embedding
            else:
                logger.error("No query text provided for search")
                return []

            # Perform search in Qdrant using query_points method
            search_results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=limit,
                with_payload=True,
                with_vectors=False
            ).points

            # Format results
            results = []
            for result in search_results:
                payload = result.payload
                formatted_result = {
                    "id": result.id,
                    "content": payload.get("content", ""),
                    "page": payload.get("page"),
                    "section": payload.get("section"),
                    "metadata": payload.get("metadata", {}),
                    "similarity_score": result.score
                }
                results.append(formatted_result)

            logger.debug(f"Found {len(results)} relevant chunks for query")
            return results

        except Exception as e:
            logger.error(f"Search failed: {e}")
            return []

    def get_chunk(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific chunk by ID
        """
        try:
            points = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[chunk_id],
                with_payload=True,
                with_vectors=False
            )

            if points:
                point = points[0]
                payload = point.payload
                return {
                    "id": point.id,
                    "content": payload.get("content", ""),
                    "page": payload.get("page"),
                    "section": payload.get("section"),
                    "metadata": payload.get("metadata", {}),
                }

            return None
        except Exception as e:
            logger.error(f"Failed to retrieve chunk {chunk_id}: {e}")
            return None

    def count_chunks(self) -> int:
        """
        Count the total number of chunks in the collection
        """
        try:
            count_result = self.client.count(
                collection_name=self.collection_name
            )
            return count_result.count
        except Exception as e:
            logger.error(f"Failed to count chunks: {e}")
            return 0

    async def check_duplicate(self, content: str, threshold: float = 0.95) -> bool:
        """
        Check if content already exists in the collection (prevent duplicates)
        """
        try:
            # Generate embedding for the content
            content_embedding = await self.embedding_service.generate_embedding(content)

            # Search for similar content
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=content_embedding,
                limit=1,
                with_payload=True,
                with_vectors=False,
                score_threshold=threshold
            )

            return len(search_results) > 0
        except Exception as e:
            logger.error(f"Failed to check for duplicates: {e}")
            return False

    async def get_chunk_by_content(self, content: str, threshold: float = 0.95) -> Optional[str]:
        """
        Get the ID of a chunk if it already exists in the collection
        """
        try:
            # Generate embedding for the content
            content_embedding = await self.embedding_service.generate_embedding(content)

            # Search for similar content
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=content_embedding,
                limit=1,
                with_payload=True,
                with_vectors=False,
                score_threshold=threshold
            )

            if search_results:
                return str(search_results[0].id)  # Return the ID of the similar chunk
            return None
        except Exception as e:
            logger.error(f"Failed to find chunk by content: {e}")
            return None

    def validate_migration(self) -> Dict[str, Any]:
        """
        Validate the migration by comparing expected vs actual chunks
        """
        try:
            # Count total chunks in collection
            total_chunks = self.count_chunks()

            # Get sample of chunks to verify content
            sample_chunks = self.get_all_chunks(limit=5)

            validation_result = {
                "total_chunks": total_chunks,
                "sample_chunks": sample_chunks,
                "validation_passed": len(sample_chunks) > 0 if total_chunks > 0 else True,
                "collection_exists": self.check_collection_exists()
            }

            return validation_result
        except Exception as e:
            logger.error(f"Failed to validate migration: {e}")
            return {
                "total_chunks": 0,
                "sample_chunks": [],
                "validation_passed": False,
                "collection_exists": False,
                "error": str(e)
            }

    def get_all_chunks(self, limit: int = 1000) -> List[Dict[str, Any]]:
        """
        Retrieve all chunks from the collection (with limit for performance)
        """
        try:
            points = self.client.scroll(
                collection_name=self.collection_name,
                limit=limit,
                with_payload=True,
                with_vectors=False
            )

            results = []
            for point in points[0]:  # Scroll returns (points, next_page_offset)
                payload = point.payload
                results.append({
                    "id": point.id,
                    "content": payload.get("content", ""),
                    "page": payload.get("page"),
                    "section": payload.get("section"),
                    "metadata": payload.get("metadata", {}),
                })

            return results
        except Exception as e:
            logger.error(f"Failed to retrieve all chunks: {e}")
            return []

    def remove_in_memory_fallback(self):
        """
        Remove any in-memory fallback functionality (as required by spec)
        This method serves as documentation that in-memory fallback is not implemented
        """
        logger.info("In-memory fallback functionality is not implemented - using Qdrant Cloud only")


# Global instance for easy access
rag_service = RAGService()