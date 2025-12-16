from sqlalchemy import Column, Integer, String, Text, DateTime
from sqlalchemy.sql import func
from ..models.database import Base
from pydantic import BaseModel
from typing import Optional
from datetime import datetime

# Database Model
class ChapterMetadata(Base):
    __tablename__ = "chapter_metadata"

    id = Column(Integer, primary_key=True, index=True)
    chapter_title = Column(String(255))
    file_path = Column(String(500))
    content_summary = Column(Text)
    paragraph_count = Column(Integer)
    created_at = Column(DateTime, default=func.now())

# Pydantic Models for API
class ChapterMetadataBase(BaseModel):
    chapter_title: str
    file_path: str
    content_summary: Optional[str] = None
    paragraph_count: Optional[int] = None

class ChapterMetadataCreate(ChapterMetadataBase):
    pass

class ChapterMetadataUpdate(BaseModel):
    chapter_title: Optional[str] = None
    file_path: Optional[str] = None
    content_summary: Optional[str] = None
    paragraph_count: Optional[int] = None

class ChapterMetadataInDB(ChapterMetadataBase):
    id: int
    created_at: datetime

    class Config:
        from_attributes = True

class ChapterMetadataPublic(BaseModel):
    id: int
    chapter_title: str
    file_path: str
    content_summary: Optional[str]
    paragraph_count: Optional[int]
    created_at: datetime

    class Config:
        from_attributes = True