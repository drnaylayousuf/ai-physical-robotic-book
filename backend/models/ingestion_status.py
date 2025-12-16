from sqlalchemy import Column, Integer, String, Text, DateTime, JSON
from sqlalchemy.sql import func
from ..models.database import Base
from pydantic import BaseModel
from typing import Optional, Dict, Any
from datetime import datetime

# Database Model
class IngestionStatus(Base):
    __tablename__ = "ingestion_status"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, nullable=False)  # ID of user who initiated the ingestion
    status = Column(String(50), default='pending')  # pending, processing, completed, failed
    total_files = Column(Integer, default=0)
    processed_files = Column(Integer, default=0)
    total_chunks = Column(Integer, default=0)
    error_message = Column(Text)
    progress_percentage = Column(Integer, default=0)
    started_at = Column(DateTime, default=func.now())
    completed_at = Column(DateTime)
    ingestion_metadata = Column(JSON)  # Additional metadata about the ingestion process

# Pydantic Models for API
class IngestionStatusBase(BaseModel):
    user_id: int
    status: str
    total_files: int
    processed_files: int
    total_chunks: int
    error_message: Optional[str] = None
    progress_percentage: int
    ingestion_metadata: Optional[Dict[str, Any]] = None

class IngestionStatusCreate(IngestionStatusBase):
    pass

class IngestionStatusUpdate(BaseModel):
    status: Optional[str] = None
    processed_files: Optional[int] = None
    total_chunks: Optional[int] = None
    error_message: Optional[str] = None
    progress_percentage: Optional[int] = None
    completed_at: Optional[datetime] = None

class IngestionStatusInDB(IngestionStatusBase):
    id: int
    started_at: datetime
    completed_at: Optional[datetime]

    class Config:
        from_attributes = True

class IngestionStatusPublic(IngestionStatusInDB):
    pass