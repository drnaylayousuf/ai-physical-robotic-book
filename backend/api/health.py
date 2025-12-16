from fastapi import APIRouter
from datetime import datetime
from pydantic import BaseModel

router = APIRouter()

class HealthResponse(BaseModel):
    status: str
    timestamp: str

@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint to verify system status
    """
    return {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat()
    }