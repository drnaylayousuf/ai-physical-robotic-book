from sqlalchemy import Column, Integer, String, DateTime, ForeignKey
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from ..models.database import Base
from pydantic import BaseModel, EmailStr
from typing import Optional
from datetime import datetime



# Database Model
class User(Base):
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    username = Column(String(255), unique=True, index=True, nullable=False)
    email = Column(String(255), unique=True, index=True, nullable=False)
    password_hash = Column(String(255), nullable=False)
    role = Column(String(50), default='user')  # Possible values: 'admin', 'moderator', 'user', 'guest'
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now())

    # Relationships
    queries = relationship("UserQuery", back_populates="user")
    selected_texts = relationship("UserSelectedText", back_populates="user")

# Pydantic Models for API
class UserBase(BaseModel):
    username: str
    email: EmailStr

class UserCreate(UserBase):
    password: str

class UserUpdate(BaseModel):
    username: Optional[str] = None
    email: Optional[EmailStr] = None
    role: Optional[str] = None

class UserInDB(UserBase):
    id: int
    role: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class UserPublic(UserBase):
    id: int
    role: str
    created_at: datetime

    class Config:
        from_attributes = True

        # i added it here

# FIX: Ensure SQLAlchemy can resolve the relationship
from .user_selected_text import UserSelectedText
