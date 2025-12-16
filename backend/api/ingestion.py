from fastapi import APIRouter, Depends, HTTPException, status, UploadFile, File
from fastapi.security import HTTPBearer
from pydantic import BaseModel
from typing import Optional, Dict, Any
import logging
from pathlib import Path
import os
from sqlalchemy.orm import Session

from ..models.rag import RAGModel
from ..models.database import get_db, SessionLocal
from ..models.user import User
from ..models.ingestion_status import IngestionStatus
from ..utils.auth import decode_access_token, get_role_from_token
from ..utils.preprocessing import TextPreprocessor
from ..config.settings import settings
from ..utils.monitoring import monitoring

router = APIRouter()
logger = logging.getLogger(__name__)

def get_db_session():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# Request/Response models
class IngestionRequest(BaseModel):
    source_path: Optional[str] = None  # Path to directory or file
    collection_name: Optional[str] = None

class IngestionResponse(BaseModel):
    status: str
    chunks_processed: int
    collection_name: str
    timestamp: str

@router.post("/ingest", response_model=IngestionResponse)
async def ingest_content(
    request: IngestionRequest,
    db: Session = Depends(get_db_session),
    token: str = Depends(HTTPBearer())
):
    """
    Upload and process book content from doc/ folder
    """
    # Decode token to get user info
    payload = decode_access_token(token.credentials)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    username = payload.get("sub")
    user_role = get_role_from_token(token.credentials) or "user"

    # Check if user has admin privileges (only admin can ingest content)
    if user_role != "admin":
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Only admin users can perform content ingestion"
        )

    try:
        # Decode token to get user ID (we need the user ID from the payload)
        payload = decode_access_token(token.credentials)
        if payload is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Get user ID from the database based on username
        username = payload.get("sub")
        user = db.query(User).filter(User.username == username).first()
        user_id = user.id if user else 1  # Default to 1 if user not found

        # Initialize RAG model
        rag_model = RAGModel()

        # Determine source path
        source_path = request.source_path or settings.BOOK_CONTENT_PATH

        # Validate source path exists
        if not os.path.exists(source_path):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Source path does not exist: {source_path}"
            )

        # Determine collection name
        collection_name = request.collection_name or settings.QDRANT_COLLECTION_NAME

        # Read content from directory
        preprocessor = TextPreprocessor()
        content_map = preprocessor.read_directory_content(source_path)

        if not content_map:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"No supported files found in {source_path}"
            )

        # Create an ingestion status record
        ingestion_status = IngestionStatus(
            user_id=user_id,
            status='processing',
            total_files=len(content_map),
            processed_files=0,
            total_chunks=0,
            progress_percentage=0,
            metadata={
                "source_path": source_path,
                "collection_name": collection_name,
                "files_to_process": list(content_map.keys())
            }
        )
        db.add(ingestion_status)
        db.commit()
        db.refresh(ingestion_status)

        # Combine all content
        combined_content = ""
        for file_path, content in content_map.items():
            combined_content += f"\n\n--- Content from {file_path} ---\n\n{content}"

        # Update status to show files are being processed
        ingestion_status.processed_files = len(content_map)
        ingestion_status.progress_percentage = 50  # Halfway through reading files
        db.commit()

        # Import the progress tracker
        from ..utils.progress_tracker import ProgressTracker

        # Create a progress tracker instance
        progress_tracker = ProgressTracker(db, ingestion_status.id)

        # Ingest the content using RAG model with progress tracking
        ingestion_result = await rag_model.ingest_content(
            combined_content,
            collection_name,
            progress_tracker=progress_tracker
        )

        # The RAG model will have updated the status, so we just need to ensure it's completed
        if ingestion_status.status != 'failed':  # Only update if not already marked as failed
            from datetime import datetime
            ingestion_status.status = 'completed'
            ingestion_status.total_chunks = ingestion_result["chunks_processed"]
            ingestion_status.progress_percentage = 100
            ingestion_status.completed_at = datetime.utcnow()

        db.commit()

        # Log the ingestion
        monitoring.log_user_action(
            username,
            "ingest_content",
            {
                "source_path": source_path,
                "collection_name": collection_name,
                "files_processed": len(content_map),
                "chunks_processed": ingestion_result["chunks_processed"]
            }
        )

        return IngestionResponse(
            status="success",
            chunks_processed=ingestion_result["chunks_processed"],
            collection_name=collection_name,
            timestamp=datetime.utcnow().isoformat()
        )

    except Exception as e:
        logger.error(f"Error during content ingestion: {e}")
        monitoring.log_error(e, "Content ingestion failed")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error during content ingestion: {str(e)}"
        )

# Alternative endpoint to upload files directly
@router.post("/ingest/upload", response_model=IngestionResponse)
async def upload_and_ingest_file(
    file: UploadFile = File(...),
    collection_name: Optional[str] = None,
    db: Session = Depends(get_db_session),
    token: str = Depends(HTTPBearer())
):
    """
    Upload and ingest a single file
    """
    # Decode token to get user info
    payload = decode_access_token(token.credentials)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    username = payload.get("sub")
    user_role = get_role_from_token(token.credentials) or "user"

    # Check if user has admin privileges
    if user_role != "admin":
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Only admin users can perform content ingestion"
        )

    try:
        # Decode token to get user ID (we need the user ID from the payload)
        payload = decode_access_token(token.credentials)
        if payload is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Get user ID from the database based on username
        username = payload.get("sub")
        user = db.query(User).filter(User.username == username).first()
        user_id = user.id if user else 1  # Default to 1 if user not found

        # Initialize RAG model
        rag_model = RAGModel()

        # Read file content
        content = await file.read()
        text_content = content.decode('utf-8')

        # Determine collection name
        collection_name = collection_name or settings.QDRANT_COLLECTION_NAME

        # Create an ingestion status record for the file upload
        ingestion_status = IngestionStatus(
            user_id=user_id,
            status='processing',
            total_files=1,
            processed_files=0,
            total_chunks=0,
            progress_percentage=0,
            metadata={
                "file_name": file.filename,
                "file_size": len(content),
                "collection_name": collection_name
            }
        )
        db.add(ingestion_status)
        db.commit()
        db.refresh(ingestion_status)

        # Update status to show file is being processed
        ingestion_status.processed_files = 1
        ingestion_status.progress_percentage = 50  # Halfway through processing
        db.commit()

        # Import the progress tracker
        from ..utils.progress_tracker import ProgressTracker

        # Create a progress tracker instance
        progress_tracker = ProgressTracker(db, ingestion_status.id)

        # Process and ingest the content with progress tracking
        ingestion_result = await rag_model.ingest_content(
            text_content,
            collection_name,
            progress_tracker=progress_tracker
        )

        # The RAG model will have updated the status, so we just need to ensure it's completed
        if ingestion_status.status != 'failed':  # Only update if not already marked as failed
            from datetime import datetime
            ingestion_status.status = 'completed'
            ingestion_status.total_chunks = ingestion_result["chunks_processed"]
            ingestion_status.progress_percentage = 100
            ingestion_status.completed_at = datetime.utcnow()

        db.commit()

        # Log the ingestion
        monitoring.log_user_action(
            username,
            "upload_and_ingest",
            {
                "file_name": file.filename,
                "file_size": len(content),
                "collection_name": collection_name,
                "chunks_processed": ingestion_result["chunks_processed"]
            }
        )

        return IngestionResponse(
            status="success",
            chunks_processed=ingestion_result["chunks_processed"],
            collection_name=collection_name,
            timestamp=datetime.utcnow().isoformat()
        )

    except Exception as e:
        logger.error(f"Error during file upload and ingestion: {e}")
        monitoring.log_error(e, "File upload and ingestion failed")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error during file upload and ingestion: {str(e)}"
        )

# Endpoint to get ingestion status
@router.get("/ingest/status")
async def get_ingestion_status(
    db: Session = Depends(get_db_session),
    token: str = Depends(HTTPBearer())
):
    """
    Get the status of content ingestion
    """
    try:
        # Decode token to get user info
        payload = decode_access_token(token.credentials)
        if payload is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )

        username = payload.get("sub")
        user_role = get_role_from_token(token.credentials) or "user"

        # Get user ID from the database based on username
        user = db.query(User).filter(User.username == username).first()
        user_id = user.id if user else 1  # Default to 1 if user not found

        # Get the most recent ingestion status for this user
        latest_ingestion = db.query(IngestionStatus).filter(
            IngestionStatus.user_id == user_id
        ).order_by(IngestionStatus.started_at.desc()).first()

        if latest_ingestion:
            return {
                "status": latest_ingestion.status,
                "total_files": latest_ingestion.total_files,
                "processed_files": latest_ingestion.processed_files,
                "total_chunks": latest_ingestion.total_chunks,
                "progress_percentage": latest_ingestion.progress_percentage,
                "started_at": latest_ingestion.started_at.isoformat(),
                "completed_at": latest_ingestion.completed_at.isoformat() if latest_ingestion.completed_at else None,
                "metadata": latest_ingestion.metadata
            }
        else:
            return {
                "status": "no_recent_ingestion",
                "message": "No recent ingestion activity found for this user"
            }

    except Exception as e:
        logger.error(f"Error getting ingestion status: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting ingestion status: {str(e)}"
        )