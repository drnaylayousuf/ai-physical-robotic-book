#!/usr/bin/env python3
"""
Migration script to migrate book content to Qdrant Cloud with Cohere embeddings
"""
import asyncio
import sys
import os
from typing import List, Dict, Any
import logging
from pathlib import Path

# Add the backend directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from backend.services.rag_service import RAGService, BookChunk, MigrationStatus
from backend.utils.content_parser import ContentParser
from backend.config.settings import settings

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def migrate_content_to_qdrant(content_dir: str) -> MigrationStatus:
    """
    Migrate book content from directory to Qdrant Cloud

    Args:
        content_dir: Directory containing book content files

    Returns:
        MigrationStatus with migration statistics
    """
    logger.info(f"Starting migration from directory: {content_dir}")

    # Initialize services
    rag_service = RAGService()
    content_parser = ContentParser()

    # Parse content from directory
    logger.info("Parsing content files...")
    chunks = content_parser.parse_directory(content_dir)

    logger.info(f"Parsed {len(chunks)} content chunks for migration")

    # Create BookChunk objects from parsed content
    book_chunks = []
    for i, chunk_data in enumerate(chunks):
        chunk = BookChunk(
            id=i,  # Use integer ID instead of string for Qdrant Cloud compatibility
            content=chunk_data["content"],
            page=chunk_data.get("page"),
            section=chunk_data.get("section", "unknown"),
            metadata=chunk_data.get("metadata", {})
        )
        book_chunks.append(chunk)

    # Perform migration
    logger.info("Starting migration to Qdrant Cloud...")
    status = await rag_service.add_chunks(book_chunks)

    logger.info(f"Migration completed. Total: {status.total_chunks}, Migrated: {status.migrated_chunks}, Errors: {status.error_count}")

    return status


async def main():
    """Main function to run the migration script"""
    import argparse

    parser = argparse.ArgumentParser(description="Migrate book content to Qdrant Cloud")
    parser.add_argument(
        "--content-dir",
        type=str,
        required=True,
        help="Directory containing book content files to migrate"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Perform a dry run without actually migrating data"
    )

    args = parser.parse_args()

    if not os.path.exists(args.content_dir):
        logger.error(f"Content directory does not exist: {args.content_dir}")
        sys.exit(1)

    if not os.path.isdir(args.content_dir):
        logger.error(f"Path is not a directory: {args.content_dir}")
        sys.exit(1)

    if args.dry_run:
        logger.info("DRY RUN: No actual migration will occur")
        logger.info(f"Would migrate content from: {args.content_dir}")
        logger.info("Environment variables check:")
        logger.info(f"  QDRANT_URL: {'SET' if settings.QDRANT_URL else 'NOT SET'}")
        logger.info(f"  QDRANT_API_KEY: {'SET' if settings.QDRANT_API_KEY else 'NOT SET'}")
        logger.info(f"  QDRANT_COLLECTION_NAME: {settings.QDRANT_COLLECTION_NAME}")
        return

    try:
        # Perform migration
        status = await migrate_content_to_qdrant(args.content_dir)

        # Print migration report
        print("\n" + "="*50)
        print("MIGRATION REPORT")
        print("="*50)
        print(f"Total chunks processed: {status.total_chunks}")
        print(f"Successfully migrated: {status.migrated_chunks}")
        print(f"Errors encountered: {status.error_count}")

        if status.errors:
            print("\nErrors:")
            for error in status.errors:
                print(f"  - Chunk {error['chunk_id']}: {error['error']}")

        print("="*50)

        if status.error_count == 0:
            logger.info("Migration completed successfully!")
            sys.exit(0)
        else:
            logger.warning(f"Migration completed with {status.error_count} errors")
            sys.exit(1)

    except Exception as e:
        logger.error(f"Migration failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())