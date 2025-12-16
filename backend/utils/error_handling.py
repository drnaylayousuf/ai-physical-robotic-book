from typing import Optional, Callable, Any
from functools import wraps
import logging
import asyncio
from ..utils.monitoring import monitoring
from fastapi import HTTPException, status
import time

logger = logging.getLogger(__name__)

class ExternalServiceError(Exception):
    """Exception raised when external services fail"""
    def __init__(self, service_name: str, original_error: Exception):
        self.service_name = service_name
        self.original_error = original_error
        super().__init__(f"{service_name} service failed: {str(original_error)}")

class CircuitBreaker:
    """
    Circuit breaker pattern implementation for external service calls
    """
    def __init__(self, failure_threshold: int = 5, recovery_timeout: int = 60):
        self.failure_threshold = failure_threshold
        self.recovery_timeout = recovery_timeout
        self.failure_count = 0
        self.last_failure_time = None
        self.state = "CLOSED"  # CLOSED, OPEN, HALF_OPEN

    def call(self, func: Callable, *args, **kwargs):
        """
        Call a function with circuit breaker protection
        """
        if self.state == "OPEN":
            # Check if it's time to attempt recovery
            if (time.time() - self.last_failure_time) >= self.recovery_timeout:
                self.state = "HALF_OPEN"
            else:
                raise ExternalServiceError("Circuit Breaker", Exception("Circuit breaker is OPEN"))

        try:
            result = func(*args, **kwargs)
            self.on_success()
            return result
        except Exception as e:
            self.on_failure()
            raise e

    def on_success(self):
        """
        Called when a call succeeds
        """
        self.failure_count = 0
        self.state = "CLOSED"

    def on_failure(self):
        """
        Called when a call fails
        """
        self.failure_count += 1
        self.last_failure_time = time.time()

        if self.failure_count >= self.failure_threshold:
            self.state = "OPEN"

def retry_on_failure(max_retries: int = 3, delay: float = 1.0, backoff: float = 2.0):
    """
    Decorator to retry function calls on failure
    """
    def decorator(func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            retries = 0
            current_delay = delay

            while retries < max_retries:
                try:
                    return await func(*args, **kwargs)
                except Exception as e:
                    retries += 1
                    if retries >= max_retries:
                        # Log the final failure
                        monitoring.log_error(e, f"Function {func.__name__} failed after {max_retries} retries")
                        raise

                    # Log the retry attempt
                    logger.warning(f"Function {func.__name__} failed, retrying in {current_delay}s... (attempt {retries}/{max_retries})")
                    monitoring.log_error(e, f"Function {func.__name__} retry attempt {retries}")

                    # Wait before retrying
                    await asyncio.sleep(current_delay)
                    current_delay *= backoff  # Exponential backoff

        return wrapper
    return decorator

def handle_external_service_errors(service_name: str):
    """
    Decorator to handle external service errors with fallback mechanisms
    """
    def decorator(func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            try:
                return await func(*args, **kwargs)
            except Exception as e:
                error_msg = f"{service_name} service error: {str(e)}"
                logger.error(error_msg)
                monitoring.log_error(e, f"{service_name} service error")

                # Log the specific error and context
                logger.info(f"Attempting fallback for {service_name} service...")

                # Re-raise as ExternalServiceError
                raise ExternalServiceError(service_name, e)

        return wrapper
    return decorator

def fallback_to_cache(func):
    """
    Decorator that provides a caching fallback when primary service fails
    """
    cache = {}

    @wraps(func)
    async def wrapper(*args, **kwargs):
        # Create a cache key based on function name and arguments
        cache_key = f"{func.__name__}_{str(args)}_{str(sorted(kwargs.items()))}"

        try:
            # Try the primary function
            result = await func(*args, **kwargs)
            # Cache the successful result
            cache[cache_key] = result
            return result
        except ExternalServiceError as e:
            # If primary service fails, try to return cached result
            if cache_key in cache:
                logger.info(f"Using cached result for {func.__name__} due to {e.service_name} failure")
                return cache[cache_key]
            else:
                # No cached result available, re-raise the error
                raise e
        except Exception as e:
            # For other exceptions, re-raise as-is
            raise e

    return wrapper

def timeout_handler(timeout_seconds: float = 30.0):
    """
    Decorator to handle function timeouts
    """
    def decorator(func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            try:
                # Use asyncio.wait_for to implement timeout
                result = await asyncio.wait_for(
                    func(*args, **kwargs),
                    timeout=timeout_seconds
                )
                return result
            except asyncio.TimeoutError:
                error_msg = f"Function {func.__name__} timed out after {timeout_seconds} seconds"
                logger.error(error_msg)
                monitoring.log_error(
                    asyncio.TimeoutError(error_msg),
                    f"Function {func.__name__} timeout"
                )
                raise HTTPException(
                    status_code=status.HTTP_408_REQUEST_TIMEOUT,
                    detail="Request timed out"
                )

        return wrapper
    return decorator

# Example usage functions
async def example_external_call():
    """
    Example of how to use the error handling utilities
    """
    # This would be a call to an external service like Gemini API
    pass

# Circuit breaker example
circuit_breaker = CircuitBreaker(failure_threshold=3, recovery_timeout=30)

async def call_with_circuit_breaker():
    """
    Example of using circuit breaker
    """
    return circuit_breaker.call(example_external_call)