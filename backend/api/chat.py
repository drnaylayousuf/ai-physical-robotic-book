from fastapi import APIRouter, HTTPException, status, Depends
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import logging

from backend.models.rag import RAGModel, RAGResponse
from backend.config.settings import settings
from backend.models.database import get_db, SessionLocal
from backend.models.user_query import UserQuery
from sqlalchemy.orm import Session
from backend.utils.cache import cache
from backend.utils.sanitization import sanitize_input, validate_question, validate_mode, sanitize_selected_text

router = APIRouter()
logger = logging.getLogger(__name__)

# Global RAG model instance for reuse to prevent timeout on first request
_rag_model_instance = None
_rag_model_initialized = False
_rag_model_initialization_error = None


def get_rag_model():
    """
    Get or create a singleton RAG model instance to prevent repeated initialization
    This helps avoid timeout issues during first request on Railway
    """
    global _rag_model_instance, _rag_model_initialized, _rag_model_initialization_error

    # If already initialized successfully, return the instance
    if _rag_model_initialized and _rag_model_instance is not None:
        return _rag_model_instance

    # If there was a previous initialization error, return cached error info
    if _rag_model_initialization_error is not None:
        logger.warning("RAG model initialization previously failed, returning cached error")
        raise _rag_model_initialization_error

    # Try to initialize the RAG model
    try:
        _rag_model_instance = RAGModel()
        _rag_model_initialized = True
        _rag_model_initialization_error = None
        logger.info("RAG model initialized successfully")
        return _rag_model_instance
    except Exception as e:
        logger.error(f"Failed to initialize RAG model: {e}")
        _rag_model_initialization_error = e
        # Re-raise the exception to indicate initialization failure
        raise

# Request/Response models
class AskRequest(BaseModel):
    question: str
    mode: str  # "full_book" or "selected_text"
    selected_text: Optional[str] = None
    page_context: Optional[Dict] = None
    user_id: Optional[str] = None

class AskResponse(BaseModel):
    response: str
    sources: List[Dict[str, Any]]
    references: List[str]

def get_db_session():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

@router.post("/ask", response_model=AskResponse)
async def ask_question(
    request: AskRequest,
    db: Session = Depends(get_db_session)
):
    """
    Handle RAG-based question answering with two modes:
    - Full-book RAG mode
    - Selected-text RAG mode (based on user-selected text)
    """
    # For public access, we'll use a default guest user
    username = "guest"
    user_role = "guest"

    # Sanitize and validate inputs
    sanitized_question = sanitize_input(request.question)
    if not validate_question(sanitized_question):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Question must be 3-1000 characters and contain at least one alphanumeric character"
        )

    if not validate_mode(request.mode):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Mode must be either 'full_book' or 'selected_text'"
        )

    # Sanitize and validate selected_text for selected_text mode
    sanitized_selected_text = None
    effective_mode = request.mode  # Use this to potentially override the mode

    if request.mode == "selected_text":
        if not request.selected_text or not request.selected_text.strip():
            # If selected_text mode is requested but no text is provided,
            # switch to full_book mode internally
            logger.info("Selected text mode requested but no text provided, switching to full_book mode")
            effective_mode = "full_book"
        else:
            temp_sanitized_text = sanitize_selected_text(request.selected_text)
            if not temp_sanitized_text.strip():
                logger.info("Selected text mode requested but text is empty after sanitization, switching to full_book mode")
                effective_mode = "full_book"
            else:
                # Valid selected text provided, proceed with selected_text mode
                sanitized_selected_text = temp_sanitized_text

    try:
        # Check cache first using sanitized question and effective mode
        cached_response = cache.get(sanitized_question, effective_mode)
        if cached_response:
            logger.info(f"Cache hit for question: {sanitized_question[:50]}...")

            # Create a default user ID for guest users (in a real system, this would come from authentication)
            user_id = 1  # This should be replaced with actual authenticated user ID when auth is implemented

            # Store the query in the database (even for cached responses to track usage)
            user_query = UserQuery(
                user_id=user_id,
                question=sanitized_question,  # Use sanitized question
                response=cached_response["response"],
                sources=cached_response["sources"]  # This should be serializable JSON
            )

            db.add(user_query)

            # If selected_text mode was used (based on effective_mode), also store the sanitized selected text
            if effective_mode == "selected_text" and sanitized_selected_text:
                from backend.models.user_selected_text import UserSelectedText

                user_selected_text = UserSelectedText(
                    user_id=user_id,
                    selected_text=sanitized_selected_text,  # Use sanitized text
                    context_metadata={
                        "question": sanitized_question,
                        "mode": effective_mode,  # Use effective mode
                        "timestamp": "current",  # This will be set by the database
                        "from_cache": True
                    }
                )

                db.add(user_selected_text)

            db.commit()
            db.refresh(user_query)

            return AskResponse(
                response=cached_response["response"],
                sources=cached_response["sources"],
                references=cached_response["references"]
            )

        # Cache miss - process normally
        logger.info(f"Cache miss for question: {sanitized_question[:50]}...")

        # Get or create the singleton RAG model instance to prevent timeout issues
        try:
            rag_model = get_rag_model()
        except Exception as e:
            logger.error(f"Failed to initialize RAG model for query: {e}")
            # Return a helpful error response instead of crashing
            return AskResponse(
                response="The RAG service is currently unavailable. Please try again later.",
                sources=[],
                references=[]
            )

        # Process the query using the RAG pipeline with sanitized inputs
        try:
            rag_response = await rag_model.process_query(
                query=sanitized_question,  # Use sanitized question
                mode=effective_mode,  # Use effective mode (may have been switched from selected_text to full_book)
                selected_text=sanitized_selected_text  # Use sanitized selected text
            )
        except Exception as e:
            logger.error(f"Error processing RAG query: {e}")
            return AskResponse(
                response="Error processing your question. Please try again later.",
                sources=[],
                references=[]
            )

        # Create a default user ID for guest users (in a real system, this would come from authentication)
        # For now, we'll use a default user_id of 1 (representing a guest user)
        user_id = 1  # This should be replaced with actual authenticated user ID when auth is implemented

        # Store the response in cache using sanitized question
        cache.set(
            sanitized_question,
            request.mode,
            {
                "response": rag_response.response,
                "sources": rag_response.sources,
                "references": rag_response.references
            },
            ttl=3600  # Cache for 1 hour
        )

        # Store the query in the database with sanitized question
        user_query = UserQuery(
            user_id=user_id,
            question=sanitized_question,  # Use sanitized question
            response=rag_response.response,
            sources=rag_response.sources  # This should be serializable JSON
        )

        db.add(user_query)

        # If selected_text mode was used, also store the sanitized selected text
        if request.mode == "selected_text" and sanitized_selected_text:
            from backend.models.user_selected_text import UserSelectedText

            user_selected_text = UserSelectedText(
                user_id=user_id,
                selected_text=sanitized_selected_text,  # Use sanitized text
                context_metadata={
                    "question": sanitized_question,  # Use sanitized question
                    "mode": request.mode,
                    "timestamp": "current",  # This will be set by the database
                    "from_cache": False
                }
            )

            db.add(user_selected_text)

        db.commit()
        db.refresh(user_query)

        return AskResponse(
            response=rag_response.response,
            sources=rag_response.sources,
            references=rag_response.references
        )

    except Exception as e:
        db.rollback()  # Rollback in case of error
        logger.error(f"Error processing RAG query: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Error processing your question"
        )


class DiagnosticResponse(BaseModel):
    collections: List[str]
    total_chunks: int
    sample_chunks: List[Dict[str, Any]]
    status: str


@router.get("/diagnostic/qdrant", response_model=DiagnosticResponse)
async def diagnostic_qdrant():
    """
    Diagnostic endpoint to check Qdrant collection status and content for troubleshooting.
    Returns collection names, number of chunks, and first 3 chunks from the collection.
    """
    try:
        from qdrant_client import QdrantClient
        from qdrant_client.http import models

        # Initialize Qdrant client
        if settings.QDRANT_API_KEY:
            client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                timeout=10
            )
        else:
            client = QdrantClient(
                url=settings.QDRANT_URL,
                timeout=10
            )

        # Get all collections
        collections_info = client.get_collections()
        collection_names = [c.name for c in collections_info.collections]

        # Get collection info for the main book chunks collection
        collection_name = settings.QDRANT_COLLECTION_NAME
        try:
            collection_info = client.get_collection(collection_name)
            total_chunks = collection_info.points_count

            # Get first 3 chunks as sample
            scroll_result = client.scroll(
                collection_name=collection_name,
                limit=3,
                with_payload=True,
                with_vectors=False
            )

            sample_chunks = []
            for point in scroll_result[0]:
                sample_chunks.append({
                    "chunk_id": str(point.id),
                    "content": point.payload.get("content", "")[:200] + "..." if len(point.payload.get("content", "")) > 200 else point.payload.get("content", ""),
                    "source": point.payload.get("source", point.payload.get("collection_name", "Unknown"))
                })

        except Exception as e:
            logger.warning(f"Collection '{collection_name}' does not exist or is empty: {e}")
            total_chunks = 0
            sample_chunks = []

        return DiagnosticResponse(
            collections=collection_names,
            total_chunks=total_chunks,
            sample_chunks=sample_chunks,
            status="healthy"
        )

    except Exception as e:
        logger.error(f"Error in diagnostic endpoint: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error in diagnostic check: {str(e)}"
        )