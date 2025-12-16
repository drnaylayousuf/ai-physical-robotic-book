from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, JSON
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from ..models.database import Base
from pydantic import BaseModel
from typing import Optional, Dict, Any
from datetime import datetime

# Database Model
class UserQuery(Base):
    __tablename__ = "user_queries"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"))
    question = Column(Text, nullable=False)
    response = Column(Text, nullable=False)
    sources = Column(JSON)  # JSONB in PostgreSQL
    timestamp = Column(DateTime, default=func.now())

    # Relationship
    user = relationship("User", back_populates="queries")

# Pydantic Models for API
class UserQueryBase(BaseModel):
    user_id: int
    question: str
    response: str
    sources: Optional[Dict[str, Any]] = None

class UserQueryCreate(UserQueryBase):
    pass

class UserQueryUpdate(BaseModel):
    response: Optional[str] = None
    sources: Optional[Dict[str, Any]] = None

class UserQueryInDB(UserQueryBase):
    id: int
    timestamp: datetime

    class Config:
        from_attributes = True

class UserQueryPublic(BaseModel):
    id: int
    user_id: int
    question: str
    response: str
    sources: Optional[Dict[str, Any]]
    timestamp: datetime

    class Config:
        from_attributes = True