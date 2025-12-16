#!/usr/bin/env python3
"""
Check Qdrant client methods
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from qdrant_client import QdrantClient
from backend.config.settings import settings

# Create a client to inspect
if settings.QDRANT_API_KEY:
    client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
        timeout=5
    )
else:
    client = QdrantClient(
        url=settings.QDRANT_URL,
        timeout=5
    )

print("Available methods in QdrantClient:")
methods = [method for method in dir(client) if not method.startswith('_')]
for method in sorted(methods):
    print(f"  - {method}")

print(f"\nHas 'search' method: {hasattr(client, 'search')}")
print(f"Has 'client' attribute: {hasattr(client, 'client')}")