# Import all models to make them discoverable to Alembic
from .database import Base
from .user import User
from .user_query import UserQuery
from .user_selected_text import UserSelectedText
from .chapter_metadata import ChapterMetadata
from .ingestion_status import IngestionStatus

# This ensures all models are registered with the Base metadata
__all__ = [
    "Base",
    "User",
    "UserQuery",
    "UserSelectedText",
    "ChapterMetadata",
    "IngestionStatus"
]