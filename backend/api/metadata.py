from fastapi import APIRouter, Depends
from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
from sqlalchemy.orm import Session
from ..models.database import get_db, SessionLocal
from ..models.chapter_metadata import ChapterMetadata

router = APIRouter()

def get_db_session():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

class ChapterInfo(BaseModel):
    id: str
    title: str
    path: str
    summary: Optional[str] = None
    paragraph_count: Optional[int] = None

class MetadataResponse(BaseModel):
    title: str
    chapters: List[ChapterInfo]

@router.get("/metadata", response_model=MetadataResponse)
async def get_book_metadata(
    db: Session = Depends(get_db_session)
):
    """
    Returns book structure and chapter information
    """
    # Fetch chapter metadata from the database
    chapters_db = db.query(ChapterMetadata).all()

    chapters = []
    for chapter in chapters_db:
        chapters.append({
            "id": str(chapter.id),
            "title": chapter.chapter_title,
            "path": chapter.file_path,
            "summary": chapter.content_summary,
            "paragraph_count": chapter.paragraph_count
        })

    # Return the book metadata with actual chapter information from the database
    return {
        "title": "Physical AI & Humanoid Robotics",
        "chapters": chapters
    }