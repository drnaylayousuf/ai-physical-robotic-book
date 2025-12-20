import os
from typing import List, Optional, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct, Batch
from dotenv import load_dotenv

load_dotenv()

class QdrantCloudService:
    """
    Service class for handling Qdrant Cloud operations
    """

    def __init__(self):
        # Get Qdrant configuration from environment
        self.url = os.getenv("QDRANT_URL")
        self.api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_chunks")

        if not self.url or not self.api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required")

        # Initialize Qdrant client with Cloud configuration
        self.client = QdrantClient(
            url=self.url,
            api_key=self.api_key,
            # Timeout configuration for Cloud
            timeout=30
        )

    def health_check(self) -> Dict[str, Any]:
        """
        Check the health of the Qdrant Cloud connection

        Returns:
            Health check response from Qdrant
        """
        try:
            # Use get_collections() to verify the connection works since health() method doesn't exist in this version
            collections = self.client.get_collections()
            return {"status": "ok", "collections_count": len(collections.collections)}
        except Exception as e:
            return {"status": "error", "error": str(e)}

    def collection_exists(self, collection_name: Optional[str] = None) -> bool:
        """
        Check if a collection exists in Qdrant Cloud

        Args:
            collection_name: Name of the collection to check (uses default if None)

        Returns:
            True if collection exists, False otherwise
        """
        try:
            collection_name = collection_name or self.collection_name
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]
            return collection_name in collection_names
        except Exception:
            return False

    def create_collection(self, vector_size: int = 1024, collection_name: Optional[str] = None):
        """
        Create a collection in Qdrant Cloud with specified vector size

        Args:
            vector_size: Size of the embedding vectors (default 1024 for Cohere)
            collection_name: Name of the collection to create (uses default if None)
        """
        collection_name = collection_name or self.collection_name

        # Define the collection configuration
        self.client.recreate_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=vector_size,
                distance=models.Distance.COSINE  # Cosine distance is good for embeddings
            ),
            # Set up optimizers for Cloud performance
            optimizers_config=models.OptimizersConfigDiff(
                memmap_threshold=20000,
                indexing_threshold=20000,
            ),
        )

    def upsert_points(self, points: List[PointStruct], collection_name: Optional[str] = None):
        """
        Insert or update points in the Qdrant collection

        Args:
            points: List of PointStruct objects to upsert
            collection_name: Name of the collection (uses default if None)
        """
        collection_name = collection_name or self.collection_name
        self.client.upsert(
            collection_name=collection_name,
            points=points
        )

    def search(self,
               vector: List[float],
               limit: int = 10,
               collection_name: Optional[str] = None,
               filters: Optional[models.Filter] = None) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in Qdrant Cloud

        Args:
            vector: Query vector to search for
            limit: Maximum number of results to return
            collection_name: Name of the collection (uses default if None)
            filters: Optional filters to apply to the search

        Returns:
            List of search results with payload and similarity scores
        """
        collection_name = collection_name or self.collection_name

        results = self.client.search(
            collection_name=collection_name,
            query_vector=vector,
            limit=limit,
            with_payload=True,  # Include the original payload data
            with_vectors=False,  # Don't return the vectors themselves
            query_filter=filters
        )

        # Format results to include payload and similarity scores
        formatted_results = []
        for result in results:
            formatted_result = {
                "id": result.id,
                "payload": result.payload,
                "score": result.score
            }
            formatted_results.append(formatted_result)

        return formatted_results

    def get_point(self, point_id: str, collection_name: Optional[str] = None) -> Optional[Dict[str, Any]]:
        """
        Get a specific point by ID from the collection

        Args:
            point_id: ID of the point to retrieve
            collection_name: Name of the collection (uses default if None)

        Returns:
            Point data if found, None otherwise
        """
        collection_name = collection_name or self.collection_name

        try:
            points = self.client.retrieve(
                collection_name=collection_name,
                ids=[point_id],
                with_payload=True,
                with_vectors=False
            )

            if points:
                point = points[0]
                return {
                    "id": point.id,
                    "payload": point.payload
                }
            return None
        except Exception:
            return None

    def delete_points(self, point_ids: List[str], collection_name: Optional[str] = None):
        """
        Delete points from the collection by their IDs

        Args:
            point_ids: List of point IDs to delete
            collection_name: Name of the collection (uses default if None)
        """
        collection_name = collection_name or self.collection_name

        self.client.delete(
            collection_name=collection_name,
            points_selector=models.PointIdsList(
                points=point_ids
            )
        )

    def count_points(self, collection_name: Optional[str] = None) -> int:
        """
        Count the total number of points in the collection

        Args:
            collection_name: Name of the collection (uses default if None)

        Returns:
            Total number of points in the collection
        """
        collection_name = collection_name or self.collection_name

        try:
            count_result = self.client.count(
                collection_name=collection_name
            )
            return count_result.count
        except Exception:
            return 0

# Global instance for easy access
qdrant_service = QdrantCloudService()