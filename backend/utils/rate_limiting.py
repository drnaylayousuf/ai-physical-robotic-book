from typing import Dict
from datetime import datetime, timedelta
import time
import threading
from fastapi import HTTPException, status
from collections import defaultdict

class RateLimiter:
    """
    Rate limiting utility to prevent API abuse
    """
    def __init__(self):
        self.requests = defaultdict(list)  # key: identifier, value: list of request timestamps
        self.lock = threading.Lock()

    def is_allowed(self, identifier: str, max_requests: int, window_seconds: int) -> bool:
        """
        Check if a request is allowed based on rate limits
        """
        with self.lock:
            now = datetime.utcnow()
            window_start = now - timedelta(seconds=window_seconds)

            # Remove old requests outside the window
            self.requests[identifier] = [
                req_time for req_time in self.requests[identifier]
                if req_time > window_start
            ]

            # Check if we're under the limit
            if len(self.requests[identifier]) < max_requests:
                # Add the current request
                self.requests[identifier].append(now)
                return True

            return False

    def get_reset_time(self, identifier: str, window_seconds: int) -> datetime:
        """
        Get the time when the rate limit will reset
        """
        with self.lock:
            if identifier in self.requests:
                oldest_request = min(self.requests[identifier])
                reset_time = oldest_request + timedelta(seconds=window_seconds)
                return reset_time
            else:
                return datetime.utcnow()

# Global rate limiter instance
rate_limiter = RateLimiter()

def check_rate_limit(identifier: str, max_requests: int = 100, window_seconds: int = 3600):
    """
    Check rate limit and raise exception if exceeded
    """
    if not rate_limiter.is_allowed(identifier, max_requests, window_seconds):
        reset_time = rate_limiter.get_reset_time(identifier, window_seconds)
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail={
                "error": "Rate limit exceeded",
                "message": f"Too many requests from this {identifier.split(':')[0]}. Limit is {max_requests} requests per {window_seconds} seconds.",
                "reset_time": reset_time.isoformat()
            }
        )

# Default rate limits
DEFAULT_USER_RATE_LIMIT = (100, 3600)  # 100 requests per hour
DEFAULT_IP_RATE_LIMIT = (1000, 3600)   # 1000 requests per hour
DEFAULT_ANON_RATE_LIMIT = (10, 3600)   # 10 requests per hour for anonymous users

def apply_rate_limit(
    max_requests: int = None,
    window_seconds: int = None,
    identifier_func = None
):
    """
    Decorator to apply rate limiting to API endpoints
    """
    def decorator(func):
        async def wrapper(*args, **kwargs):
            # Get identifier (could be user ID, IP, etc.)
            identifier = identifier_func(*args, **kwargs) if identifier_func else "default"

            # Use default limits if not specified
            max_req = max_requests or DEFAULT_USER_RATE_LIMIT[0]
            window_sec = window_seconds or DEFAULT_USER_RATE_LIMIT[1]

            check_rate_limit(identifier, max_req, window_sec)

            return await func(*args, **kwargs)
        return wrapper
    return decorator

# Example identifier functions
def get_user_identifier(*args, **kwargs):
    """
    Example function to get user identifier from request
    """
    # In a real implementation, this would extract user info from the request
    request = kwargs.get('request') or (args[0] if args else None)
    if hasattr(request, 'user') and request.user:
        return f"user:{request.user.id}"
    return "user:anonymous"

def get_ip_identifier(*args, **kwargs):
    """
    Example function to get IP identifier from request
    """
    # In a real implementation, this would extract IP from the request
    request = kwargs.get('request') or (args[0] if args else None)
    if hasattr(request, 'client') and request.client:
        return f"ip:{request.client.host}"
    return "ip:unknown"