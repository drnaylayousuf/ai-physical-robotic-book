from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware
from starlette.middleware.trustedhost import TrustedHostMiddleware
from starlette.middleware import Middleware
from backend.api import health, metadata, auth, chat, ingestion, admin
from backend.config.settings import settings
import logging
from datetime import datetime

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app instance
app = FastAPI(
    title="RAG Chatbot API for Physical AI and Humanoid Robotics Book",
    description="API for interacting with the RAG chatbot system",
    version="1.0.0",
)

# Add security headers using response middleware
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response
from starlette.requests import Request
import time

class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request: Request, call_next):
        response = await call_next(request)
        # Add security headers
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"  # Or "SAMEORIGIN" if needed
        response.headers["X-XSS-Protection"] = "1; mode=block"
        response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"
        response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"
        response.headers["Content-Security-Policy"] = "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; font-src 'self'; connect-src 'self'; frame-ancestors 'none';"
        return response

# Add security middleware
app.add_middleware(SecurityHeadersMiddleware)

# Add trusted host middleware to prevent HTTP Host Header attacks
app.add_middleware(
    TrustedHostMiddleware,
    allowed_hosts=[
        "localhost",
        "127.0.0.1",
        "0.0.0.0",
        # Add your production domains here
        # ".yourdomain.com"  # For subdomains
       "*.railway.app",  # Allow all railway.app domains
       "ai-physical-robotic-book-production.up.railway.app",  # Your specific Railway domain
    ]
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Standard frontend port
        "http://127.0.0.1:3000",  # Alternative localhost format
        "http://localhost:3001",  # Alternative frontend port
        "http://localhost:3002",  # Alternative frontend port
        "http://localhost:3003",  # Alternative frontend port
        "http://localhost:8080",  # Alternative frontend port
        "http://localhost:8081",  # Alternative frontend port
        "http://localhost:5173",  # Vite default port
        "http://localhost:3004",  # Claude/Speckit likely port
        "https://ai-physical-robotic-book-production.up.railway.app",  # Your Railway deployment URL
        "https://*.railway.app",  # Allow all railway.app domains
        "https://*.vercel.app",  # Allow all Vercel domains (for frontend)
        "https://ai-physical-robotic-book.vercel.app",  # Your specific Vercel URL
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Expose headers for frontend access
    expose_headers=["Access-Control-Allow-Origin", "Access-Control-Allow-Credentials"]
)

# Include API routes
app.include_router(health.router, prefix="/api", tags=["health"])
app.include_router(metadata.router, prefix="/api", tags=["metadata"])
app.include_router(chat.router, prefix="/api", tags=["chat"])
app.include_router(ingestion.router, prefix="/api", tags=["ingestion"])
app.include_router(admin.router, prefix="/api", tags=["admin"])

@app.on_event("startup")
async def startup_event():
    """
    Log startup information (don't initialize RAG services to avoid blocking)
    """
    logger.info("RAG Chatbot API starting up...")

    # Log environment information
    import os
    logger.info(f"Environment: Running on Railway")
    logger.info(f"Port: {os.environ.get('PORT', 8000)}")
    logger.info(f"Host: {os.environ.get('HOST', '0.0.0.0')}")

    # Don't initialize RAG services during startup to avoid blocking
    # RAG services will be initialized lazily when first requested
    logger.info("RAG Chatbot API startup completed successfully (RAG services will initialize on first use)")


@app.get("/")
async def root():
    """
    Root endpoint for the API - also serves as health check
    """
    return {
        "message": "RAG Chatbot API for Physical AI and Humanoid Robotics Book",
        "status": "running",
        "timestamp": datetime.utcnow().isoformat()
    }

@app.get("/healthz")
async def healthz():
    """
    Kubernetes-style health check endpoint for Railway
    """
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    import os

    # Get port from environment variable (for Railway) or default to 8000
    port = int(os.environ.get("PORT", 8000))

    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=port,
        reload=False  # Disable reload in production
    )