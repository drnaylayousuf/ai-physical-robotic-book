from pydantic import BaseModel
from typing import List, Optional, Dict

class CollectionInfo(BaseModel):
    name: str
    vector_count: int
    indexed: bool
    indexed_fields: List[str]
    size: Optional[str] = None

class DiagnosticResponse(BaseModel):
    status: str
    collections: List[CollectionInfo]
    qdrant_cloud_info: Dict
    timestamp: str