from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime
from enum import Enum


class ServiceName(str, Enum):
    """Enumeration of service names for dependency tracking."""
    VECTOR_DATABASE = "vector_database"
    EMBEDDING_SERVICE = "embedding_service"


class DependencyStatus(BaseModel):
    """Model representing the availability status of external services."""

    service_name: ServiceName = Field(
        ...,
        description="String representing the service name ('vector_database', 'embedding_service')"
    )

    is_available: bool = Field(
        ...,
        description="Boolean indicating whether the service is accessible"
    )

    details: Optional[str] = Field(
        default=None,
        description="Optional string with additional information about service status"
    )

    last_checked: str = Field(
        default_factory=lambda: datetime.utcnow().isoformat() + "Z",
        description="ISO 8601 datetime string of when the status was last verified"
    )


class RetrievedChunk(BaseModel):
    """Model representing a retrieved content chunk (duplicate from response.py for consistency)."""

    chunk_id: str = Field(
        ...,
        description="String identifier for the specific content chunk",
        example="chunk_001"
    )

    content: str = Field(
        ...,
        description="String containing the actual text content of the chunk",
        example="Humanoid robots are robots with a humanoid body plan..."
    )

    score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Float representing the relevance score/confidence (0.0 to 1.0)",
        example=0.85
    )

    source: str = Field(
        ...,
        description="String containing metadata about the original source document",
        example="chapter_3_humanoid_design.pdf"
    )