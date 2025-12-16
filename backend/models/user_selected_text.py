from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, JSON
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from ..models.database import Base
from pydantic import BaseModel
from typing import Optional, Dict, Any
from datetime import datetime

# Database Model
class UserSelectedText(Base):
    __tablename__ = "user_selected_text"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"))
    selected_text = Column(Text, nullable=False)
    context_metadata = Column(JSON)  # JSONB in PostgreSQL
    timestamp = Column(DateTime, default=func.now())

    # Relationship
    user = relationship("User", back_populates="selected_texts")

# Pydantic Models for API
class UserSelectedTextBase(BaseModel):
    user_id: int
    selected_text: str
    context_metadata: Optional[Dict[str, Any]] = None

class UserSelectedTextCreate(UserSelectedTextBase):
    pass

class UserSelectedTextUpdate(BaseModel):
    selected_text: Optional[str] = None
    context_metadata: Optional[Dict[str, Any]] = None

class UserSelectedTextInDB(UserSelectedTextBase):
    id: int
    timestamp: datetime

    class Config:
        from_attributes = True

class UserSelectedTextPublic(BaseModel):
    id: int
    user_id: int
    selected_text: str
    context_metadata: Optional[Dict[str, Any]]
    timestamp: datetime

    class Config:
        from_attributes = True