import asyncio
import hashlib
from typing import Optional, Dict, Any
from datetime import datetime, timedelta
from ..config.settings import settings

class SimpleCache:
    """
    Simple in-memory cache for storing frequently asked questions and their responses
    """

    def __init__(self):
        self.cache = {}
        self.access_times = {}  # Track access times for LRU eviction
        self.max_size = 1000  # Maximum number of cached items
        self.default_ttl = 3600  # Default TTL in seconds (1 hour)

    def _get_cache_key(self, question: str, mode: str) -> str:
        """
        Generate a cache key based on question and mode
        """
        key_string = f"{question.lower().strip()}_{mode}"
        return hashlib.md5(key_string.encode()).hexdigest()

    def get(self, question: str, mode: str = "full_book") -> Optional[Dict[str, Any]]:
        """
        Get cached response for a question
        """
        key = self._get_cache_key(question, mode)

        if key in self.cache:
            item = self.cache[key]
            # Check if the item has expired
            if datetime.now() < item['expires_at']:
                # Update access time for LRU
                self.access_times[key] = datetime.now()
                return item['value']
            else:
                # Remove expired item
                del self.cache[key]
                del self.access_times[key]

        return None

    def set(self, question: str, mode: str, value: Dict[str, Any], ttl: Optional[int] = None) -> None:
        """
        Set a cached response for a question
        """
        key = self._get_cache_key(question, mode)

        if ttl is None:
            ttl = self.default_ttl

        # Remove oldest item if cache is full
        if len(self.cache) >= self.max_size:
            self._evict_lru()

        self.cache[key] = {
            'value': value,
            'expires_at': datetime.now() + timedelta(seconds=ttl)
        }
        self.access_times[key] = datetime.now()

    def _evict_lru(self):
        """
        Remove the least recently used item
        """
        if not self.access_times:
            return

        lru_key = min(self.access_times.keys(), key=lambda k: self.access_times[k])
        del self.cache[lru_key]
        del self.access_times[lru_key]

    def clear(self):
        """
        Clear the entire cache
        """
        self.cache.clear()
        self.access_times.clear()

# Global cache instance
cache = SimpleCache()