from fastapi import APIRouter
from datetime import datetime
import logging

# Import the Qdrant service
from backend.utils.qdrant_client import qdrant_service
from backend.services.rag_service import RAGService
from backend.models.health import HealthResponse, QdrantHealthResponse
from backend.models.diagnostic import CollectionInfo, DiagnosticResponse

router = APIRouter()
logger = logging.getLogger(__name__)

@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Basic health check endpoint to verify system status
    """
    return {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat()
    }

@router.get("/health/qdrant", response_model=QdrantHealthResponse)
async def qdrant_cloud_health():
    """
    Health check endpoint specifically for Qdrant Cloud connectivity
    """
    try:
        # Perform health check on Qdrant service
        health_status = qdrant_service.health_check()

        if health_status.get("status") == "ok":
            return {
                "status": "healthy",
                "message": "Successfully connected to Qdrant Cloud",
                "timestamp": datetime.utcnow().isoformat(),
                "qdrant_cloud_status": "connected"
            }
        else:
            return {
                "status": "unhealthy",
                "message": f"Failed to connect to Qdrant Cloud: {health_status.get('error', 'Unknown error')}",
                "timestamp": datetime.utcnow().isoformat(),
                "qdrant_cloud_status": "disconnected"
            }
    except Exception as e:
        logger.error(f"Error in Qdrant Cloud health check: {str(e)}")
        return {
            "status": "unhealthy",
            "message": f"Failed to connect to Qdrant Cloud: {str(e)}",
            "timestamp": datetime.utcnow().isoformat(),
            "qdrant_cloud_status": "disconnected"
        }

@router.get("/diagnostic/qdrant", response_model=DiagnosticResponse)
async def qdrant_diagnostic():
    """
    Diagnostic endpoint to report collection status and vector information
    """
    try:
        # Get collections information
        collections_response = qdrant_service.client.get_collections()
        collections = []

        for collection in collections_response.collections:
            # Get detailed collection info
            collection_info = qdrant_service.client.get_collection(collection.name)

            collection_detail = CollectionInfo(
                name=collection.name,
                vector_count=collection_info.points_count,
                indexed=True,  # Qdrant collections are typically indexed
                indexed_fields=[]  # Would need to inspect the collection's vector params
            )
            collections.append(collection_detail)

        # Get Qdrant server info
        try:
            # Get cluster info if available
            cluster_info = qdrant_service.client.get_cluster()
            qdrant_info = {
                "version": getattr(cluster_info, 'version', 'unknown'),
                "collections_count": len(collections),
                "total_vectors": sum(c.vector_count for c in collections)
            }
        except:
            # If cluster info not available, provide basic info
            qdrant_info = {
                "collections_count": len(collections),
                "total_vectors": sum(c.vector_count for c in collections)
            }

        return {
            "status": "success",
            "collections": [c.dict() for c in collections],
            "qdrant_cloud_info": qdrant_info,
            "timestamp": datetime.utcnow().isoformat()
        }
    except Exception as e:
        logger.error(f"Error in Qdrant diagnostic: {str(e)}")
        return {
            "status": "error",
            "collections": [],
            "qdrant_cloud_info": {},
            "message": f"Failed to get diagnostic information: {str(e)}",
            "timestamp": datetime.utcnow().isoformat()
        }