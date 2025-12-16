import logging
import traceback
from typing import Callable, Dict, Any
from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from pydantic import BaseModel, Field
from datetime import datetime
import uuid


# Define error response model
class ErrorResponse(BaseModel):
    """Standard error response format."""
    detail: str = Field(..., description="Human-readable error message")
    error_code: str = Field(None, description="Machine-readable error code")
    timestamp: str = Field(default_factory=lambda: datetime.utcnow().isoformat() + "Z")
    request_id: str = Field(default_factory=lambda: str(uuid.uuid4()))


# Custom exception classes
class RAGException(Exception):
    """Base exception for RAG system errors."""
    def __init__(self, detail: str, error_code: str = None):
        self.detail = detail
        self.error_code = error_code
        super().__init__(detail)


class VectorDatabaseException(RAGException):
    """Exception for vector database related errors."""
    def __init__(self, detail: str):
        super().__init__(detail, "VECTOR_DB_ERROR")


class EmbeddingServiceException(RAGException):
    """Exception for embedding service related errors."""
    def __init__(self, detail: str):
        super().__init__(detail, "EMBEDDING_SERVICE_ERROR")


class DependencyUnavailableException(RAGException):
    """Exception when required dependencies are unavailable."""
    def __init__(self, detail: str):
        super().__init__(detail, "DEPENDENCY_UNAVAILABLE")


class InvalidRequestException(RAGException):
    """Exception for invalid request parameters."""
    def __init__(self, detail: str):
        super().__init__(detail, "INVALID_REQUEST")


# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def add_error_handlers(app: FastAPI) -> None:
    """Add centralized exception handlers to the FastAPI app."""

    @app.exception_handler(RAGException)
    async def handle_rag_exception(request: Request, exc: RAGException):
        """Handle RAG-specific exceptions."""
        request_id = str(uuid.uuid4())

        logger.error(
            f"RAGException: {exc.detail} | Request ID: {request_id} | Error Code: {exc.error_code}",
            extra={"request_id": request_id}
        )

        error_response = ErrorResponse(
            detail=exc.detail,
            error_code=exc.error_code,
            request_id=request_id
        )

        # Map error codes to appropriate HTTP status codes
        status_code = 500
        if exc.error_code == "INVALID_REQUEST":
            status_code = 400
        elif exc.error_code in ["VECTOR_DB_ERROR", "EMBEDDING_SERVICE_ERROR", "DEPENDENCY_UNAVAILABLE"]:
            status_code = 503

        return JSONResponse(
            status_code=status_code,
            content=error_response.model_dump()
        )

    @app.exception_handler(HTTPException)
    async def handle_http_exception(request: Request, exc: HTTPException):
        """Handle standard HTTP exceptions."""
        request_id = str(uuid.uuid4())

        logger.warning(
            f"HTTPException: {exc.detail} | Status: {exc.status_code} | Request ID: {request_id}",
            extra={"request_id": request_id}
        )

        error_response = ErrorResponse(
            detail=exc.detail,
            error_code=f"HTTP_{exc.status_code}",
            request_id=request_id
        )

        return JSONResponse(
            status_code=exc.status_code,
            content=error_response.model_dump()
        )

    @app.exception_handler(RequestValidationError)
    async def handle_validation_error(request: Request, exc: RequestValidationError):
        """Handle request validation errors."""
        request_id = str(uuid.uuid4())

        # Extract the first error message for the detail field
        if exc.errors():
            first_error = exc.errors()[0]
            error_msg = first_error.get("msg", "Validation error")
            field_location = " -> ".join(str(loc) for loc in first_error.get("loc", []))
            detail_msg = f"{error_msg} (Field: {field_location})"
        else:
            detail_msg = "Request validation failed"

        logger.warning(
            f"Validation error: {detail_msg} | Request ID: {request_id}",
            extra={"request_id": request_id}
        )

        error_response = ErrorResponse(
            detail=detail_msg,
            error_code="VALIDATION_ERROR",
            request_id=request_id
        )

        return JSONResponse(
            status_code=422,
            content=error_response.model_dump()
        )

    @app.exception_handler(Exception)
    async def handle_general_exception(request: Request, exc: Exception):
        """Handle all other exceptions."""
        request_id = str(uuid.uuid4())

        logger.error(
            f"Unexpected error: {str(exc)} | Request ID: {request_id} | Traceback: {traceback.format_exc()}",
            extra={"request_id": request_id}
        )

        # Check if the error message contains dependency-related terms
        exc_msg = str(exc).lower()
        if "dependency" in exc_msg or "unavailable" in exc_msg or "connection" in exc_msg or "access" in exc_msg:
            # For dependency-related errors, provide more specific message
            detail_msg = str(exc)
        else:
            # For security, don't expose internal error details to the user for other errors
            detail_msg = "An unexpected error occurred while processing your request"

        error_response = ErrorResponse(
            detail=detail_msg,
            error_code="INTERNAL_ERROR",
            request_id=request_id
        )

        return JSONResponse(
            status_code=500,
            content=error_response.model_dump()
        )


def log_error(request: Request, exc: Exception, context: str = "") -> str:
    """Log an error with request context and return a request ID for correlation."""
    request_id = str(uuid.uuid4())

    logger.error(
        f"{context} | Error: {str(exc)} | Request ID: {request_id} | Path: {request.url.path} | Method: {request.method}",
        extra={"request_id": request_id}
    )

    return request_id


def structured_log(level: str, message: str, **kwargs) -> None:
    """Log a structured message with additional context."""
    if level.upper() == "ERROR":
        logger.error(message, extra=kwargs)
    elif level.upper() == "WARNING":
        logger.warning(message, extra=kwargs)
    elif level.upper() == "INFO":
        logger.info(message, extra=kwargs)
    else:
        logger.debug(message, extra=kwargs)