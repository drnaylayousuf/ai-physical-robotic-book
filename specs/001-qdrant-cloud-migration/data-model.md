# Data Model: Qdrant Cloud Migration

## Book Chunk
- **Fields**:
  - id: string (unique identifier for the chunk)
  - content: string (the actual text content of the book chunk)
  - page: integer (page number reference from the book)
  - section: string (section or chapter reference)
  - embedding: array[float] (vector representation from Cohere)
  - metadata: object (additional metadata like source, creation date)
  - created_at: datetime (timestamp of creation)
  - updated_at: datetime (timestamp of last update)

- **Validation**:
  - Content must not be empty
  - ID must be unique across all chunks
  - Embedding must have consistent dimensions (1024 for Cohere)

- **Relationships**: None (standalone entity stored in Qdrant)

## Qdrant Collection Schema
- **Fields**:
  - payload: object (containing Book Chunk fields except embedding)
  - vector: array[float] (the embedding vector)
  - id: string (the chunk ID)

- **Validation**:
  - Vector dimensions must match Cohere embedding size
  - Payload must contain required Book Chunk metadata

## User Query
- **Fields**:
  - query_text: string (the user's question)
  - query_embedding: array[float] (vector representation from Cohere)
  - similarity_threshold: float (minimum similarity for results)

- **Validation**:
  - Query text must not be empty
  - Similarity threshold between 0.0 and 1.0

## Migration Status
- **Fields**:
  - total_chunks: integer (total number of book chunks processed)
  - migrated_chunks: integer (number of chunks successfully migrated)
  - error_count: integer (number of chunks with errors during migration)
  - errors: array[object] (details of any errors encountered)

- **Validation**:
  - All counts must be non-negative
  - Migrated chunks must not exceed total chunks