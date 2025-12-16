import os
from typing import Optional
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # API Configuration
    api_host: str = "0.0.0.0"
    api_port: int = 8000
    api_debug: bool = False

    # Qdrant Configuration
    qdrant_url: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key: Optional[str] = os.getenv("QDRANT_API_KEY")
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "book_chunks")

    # Cohere Configuration
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")

    # Google Generative AI Configuration
    gemini_api_key: str = os.getenv("GEMINI_API_KEY", "")

    # Service validation cache settings
    dependency_validation_cache_ttl: int = 5  # seconds

    class Config:
        env_file = ".env"


# Create a single instance of settings
settings = Settings()