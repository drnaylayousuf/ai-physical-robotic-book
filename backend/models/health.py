from pydantic import BaseModel
from typing import List, Optional, Dict

class HealthResponse(BaseModel):
    status: str
    timestamp: str

class QdrantHealthResponse(BaseModel):
    status: str
    message: str
    timestamp: str
    qdrant_cloud_status: str