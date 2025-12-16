from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .middleware.error_handler import add_error_handlers
from .routes.rag import router as rag_router
from src.config.settings import settings


def create_app() -> FastAPI:
    """Create and configure the FastAPI application."""
    app = FastAPI(
        title="RAG System API",
        description="API for RAG-based question answering with error handling",
        version="1.0.0",
        debug=settings.api_debug,
    )

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  # In production, configure specific origins
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Include routers
    app.include_router(rag_router, prefix="/api", tags=["rag"])

    # Add centralized error handlers
    add_error_handlers(app)

    return app


app = create_app()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.api.main:app",
        host=settings.api_host,
        port=settings.api_port,
        reload=True,
    )