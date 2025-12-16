import logging
from datetime import datetime
from typing import Optional
import time
from functools import wraps
from fastapi import Request

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class Monitoring:
    """
    Monitoring and logging utilities for the RAG chatbot system
    """

    @staticmethod
    def log_request(request: Request, response_status: int, execution_time: float):
        """
        Log request information
        """
        logger.info(
            f"Request: {request.method} {request.url.path} | "
            f"Status: {response_status} | "
            f"Time: {execution_time:.3f}s | "
            f"IP: {request.client.host if request.client else 'unknown'}"
        )

    @staticmethod
    def log_user_action(user_id: str, action: str, details: Optional[dict] = None):
        """
        Log user actions for analytics and monitoring
        """
        log_data = {
            "user_id": user_id,
            "action": action,
            "timestamp": datetime.utcnow().isoformat(),
            "details": details or {}
        }
        logger.info(f"User action: {log_data}")

    @staticmethod
    def log_error(error: Exception, context: Optional[str] = None):
        """
        Log error with context
        """
        logger.error(
            f"Error: {str(error)} | "
            f"Context: {context or 'unknown'} | "
            f"Time: {datetime.utcnow().isoformat()}"
        )

    @staticmethod
    def log_performance(endpoint: str, execution_time: float, status: str = "success"):
        """
        Log performance metrics for an endpoint
        """
        logger.info(
            f"Performance: {endpoint} | "
            f"Time: {execution_time:.3f}s | "
            f"Status: {status}"
        )

def monitor_endpoint(func):
    """
    Decorator to monitor endpoint performance
    """
    @wraps(func)
    async def wrapper(*args, **kwargs):
        start_time = time.time()
        try:
            result = await func(*args, **kwargs)
            execution_time = time.time() - start_time
            Monitoring.log_performance(func.__name__, execution_time, "success")
            return result
        except Exception as e:
            execution_time = time.time() - start_time
            Monitoring.log_performance(func.__name__, execution_time, "error")
            Monitoring.log_error(e, f"Error in {func.__name__}")
            raise
    return wrapper

# Initialize monitoring instance
monitoring = Monitoring()