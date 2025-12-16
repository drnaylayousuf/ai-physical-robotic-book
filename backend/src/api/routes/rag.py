from fastapi import APIRouter, Depends, HTTPException
from typing import Optional
import logging
import uuid
from datetime import datetime

from src.models.request import RAGRequest
from src.models.response import RAGResponse, ErrorResponse
from src.services.rag_service import RAGService
from src.services.dependency_checker import DependencyChecker
from src.api.middleware.error_handler import (
    RAGException,
    InvalidRequestException,
    DependencyUnavailableException,
    structured_log
)

router = APIRouter()

# Initialize services
rag_service = RAGService()
dependency_checker = DependencyChecker()

# Configure logging
logger = logging.getLogger(__name__)


@router.post("/ask",
             response_model=RAGResponse,
             responses={
                 200: {"description": "Successful response with answer and sources"},
                 400: {"model": ErrorResponse, "description": "Invalid request parameters"},
                 422: {"model": ErrorResponse, "description": "Request parameters failed validation"},
                 500: {"model": ErrorResponse, "description": "Internal server error"},
                 503: {"model": ErrorResponse, "description": "Service unavailable - dependencies not ready"}
             })
async def ask_question(request: RAGRequest):
    """
    Submit a question to the RAG system for processing.

    This endpoint processes user questions using Retrieval-Augmented Generation,
    with comprehensive error handling and dependency validation.
    """
    request_id = str(uuid.uuid4())

    try:
        # Log the incoming request
        structured_log(
            "INFO",
            f"Processing RAG request | Question: {request.question[:50]}... | Mode: {request.mode} | Request ID: {request_id}",
            request_id=request_id,
            question=request.question,
            mode=request.mode.value
        )

        # Validate the request parameters
        if not request.question or not request.question.strip():
            raise InvalidRequestException("Question cannot be empty or contain only whitespace")

        # Validate mode is one of the allowed values
        if request.mode.value not in ["full_book", "selected_text"]:
            raise InvalidRequestException(f"Invalid mode: {request.mode}. Must be 'full_book' or 'selected_text'")

        # Check dependencies before processing (for modes that require them)
        # In test environments, we may skip dependency checks to allow testing of processing logic
        import os
        skip_dependency_check = os.getenv("SKIP_DEPENDENCY_CHECK", "").lower() == "true"

        if not skip_dependency_check:
            if request.mode == "full_book":
                # For full book mode, check if vector database is available
                if not await dependency_checker.check_vector_database():
                    raise DependencyUnavailableException(
                        "Vector database dependency is unavailable. Please check if Qdrant is running and accessible."
                    )

            # For selected_text mode, we still need embedding service
            if request.mode == "selected_text" and request.selected_text is not None:
                if not await dependency_checker.check_embedding_service():
                    raise DependencyUnavailableException(
                        "Embedding service dependency is unavailable. Please check Cohere API access."
                    )

        # Process the request using the RAG service
        result = await rag_service.process_query(request)

        structured_log(
            "INFO",
            f"RAG request completed successfully | Request ID: {request_id}",
            request_id=request_id
        )

        return result

    except RAGException:
        # Re-raise RAG-specific exceptions to be handled by middleware
        raise
    except HTTPException:
        # Re-raise HTTP exceptions to be handled by middleware
        raise
    except Exception as e:
        # Check if the error message contains dependency-related terms
        exc_msg = str(e).lower()
        if "dependency" in exc_msg or "unavailable" in exc_msg or "connection" in exc_msg or "access" in exc_msg:
            # For dependency-related errors, log and raise an HTTP exception with the original message
            # This allows for descriptive error messages while maintaining security
            structured_log(
                "ERROR",
                f"Dependency-related error in ask endpoint | Error: {str(e)} | Request ID: {request_id}",
                request_id=request_id,
                error=str(e)
            )

            # Raise an HTTP exception with the original message for dependency errors
            raise HTTPException(
                status_code=503,  # Service Unavailable for dependency issues
                detail=str(e)
            )
        else:
            # For other unexpected errors, log and return generic message
            structured_log(
                "ERROR",
                f"Unexpected error in ask endpoint | Error: {str(e)} | Request ID: {request_id}",
                request_id=request_id,
                error=str(e)
            )

            # For security, don't expose internal error details to the user
            raise HTTPException(
                status_code=500,
                detail="An unexpected error occurred while processing your request"
            )


@router.get("/health",
            responses={
                200: {"description": "Health check successful"},
                503: {"model": ErrorResponse, "description": "One or more dependencies unavailable"}
            })
async def health_check():
    """
    Check the health status of the RAG system and its dependencies.
    """
    request_id = str(uuid.uuid4())

    try:
        structured_log(
            "INFO",
            f"Health check requested | Request ID: {request_id}",
            request_id=request_id
        )

        # Check all dependencies
        vector_db_available = await dependency_checker.check_vector_database()
        embedding_service_available = await dependency_checker.check_embedding_service()

        # Prepare dependencies status
        dependencies = {
            "vector_database": {
                "status": "available" if vector_db_available else "unavailable",
                "last_checked": datetime.utcnow().isoformat() + "Z"
            },
            "embedding_service": {
                "status": "available" if embedding_service_available else "unavailable",
                "last_checked": datetime.utcnow().isoformat() + "Z"
            }
        }

        # Overall status is healthy if all critical dependencies are available
        overall_status = "healthy" if vector_db_available and embedding_service_available else "degraded"

        structured_log(
            "INFO",
            f"Health check completed | Status: {overall_status} | Request ID: {request_id}",
            request_id=request_id,
            overall_status=overall_status
        )

        return {
            "status": overall_status,
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "dependencies": dependencies
        }

    except Exception as e:
        structured_log(
            "ERROR",
            f"Health check failed | Error: {str(e)} | Request ID: {request_id}",
            request_id=request_id,
            error=str(e)
        )

        # Return a 503 if we can't even perform a health check
        raise HTTPException(
            status_code=503,
            detail="Unable to perform health check due to internal error"
        )


# Additional endpoints can be added here as needed