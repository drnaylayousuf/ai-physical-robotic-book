from pydantic import BaseModel, Field, validator
from typing import Optional
from enum import Enum


class RAGMode(str, Enum):
    """Enumeration of RAG processing modes."""
    FULL_BOOK = "full_book"
    SELECTED_TEXT = "selected_text"


class RAGRequest(BaseModel):
    """Request model for RAG system with validation."""

    question: str = Field(
        ...,
        description="The user's question to be answered",
        min_length=1,
        example="What are humanoid robots used for?"
    )

    mode: RAGMode = Field(
        default=RAGMode.FULL_BOOK,
        description="The processing mode - 'full_book' to search entire collection, 'selected_text' to process provided text"
    )

    selected_text: Optional[str] = Field(
        default=None,
        description="Text provided by user for selected_text mode (can be null)",
        example="Humanoid robots are robots with a humanoid body plan, i.e. they have a head, a torso, two arms, and two legs"
    )

    class Config:
        # Allow the model to handle null values for optional fields
        allow_population_by_field_name = True

    @validator('question')
    def validate_question(cls, v):
        """Validate that question is not empty or just whitespace."""
        if not v or not v.strip():
            raise ValueError('Question cannot be empty or contain only whitespace')
        return v.strip()

    @validator('selected_text')
    def validate_selected_text(cls, v):
        """Validate selected_text field."""
        if v is not None:
            return v.strip() if v.strip() else None
        return v