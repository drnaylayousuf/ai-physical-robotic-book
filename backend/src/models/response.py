from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime
import uuid


class RetrievedChunk(BaseModel):
    """Model representing a retrieved content chunk."""

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


class RAGResponse(BaseModel):
    """Response model for successful RAG requests."""

    response: str = Field(
        ...,
        description="String representing the generated answer to the user's question",
        example="Humanoid robots are used for various purposes including research, assistance, and entertainment..."
    )

    sources: List[RetrievedChunk] = Field(
        default_factory=list,
        description="Array of source objects containing retrieved context chunks"
    )

    references: List[str] = Field(
        default_factory=list,
        description="Array of reference strings for citations"
    )


class ErrorResponse(BaseModel):
    """Standard error response format."""

    detail: str = Field(
        ...,
        description="Human-readable error message",
        example="Vector database is not available. Please check if Qdrant is running and accessible."
    )

    error_code: Optional[str] = Field(
        default=None,
        description="Optional machine-readable error code",
        example="VECTOR_DB_UNAVAILABLE"
    )

    timestamp: str = Field(
        default_factory=lambda: datetime.utcnow().isoformat() + "Z",
        description="ISO 8601 datetime string of when the error occurred"
    )

    request_id: str = Field(
        default_factory=lambda: str(uuid.uuid4()),
        description="Optional string for request correlation and debugging"
    )