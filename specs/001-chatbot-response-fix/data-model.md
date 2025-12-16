# Data Model: Improved Chatbot Response Consistency and Functionality

## RAGResponse Entity
**Description**: Represents the structured response from the RAG system containing response text, sources, and references

**Fields**:
- response: string - The generated response text from the AI model
- sources: List[Dict] - Collection of source documents/chunks used to generate the response
  - chunk_id: string - Unique identifier for the document chunk
  - content: string - The actual content of the chunk
  - score: float - Relevance score of the chunk to the query
  - source: string - Source metadata (e.g., document name, URL)
- references: List[string] - List of unique source references for attribution

## QueryContext Entity
**Description**: Represents the combined context from both selected text and full database search, including relevance scores and source metadata

**Fields**:
- selected_text_chunks: List[Dict] - Chunks from user-provided selected text
  - content: string - The text content
  - similarity_score: float - Cosine similarity to the query
- database_chunks: List[Dict] - Chunks from the full database search
  - content: string - The text content
  - score: float - Vector similarity score
  - source: string - Source document identifier
- combined_context: List[Dict] - Merged and ranked context chunks
  - content: string - The text content
  - score: float - Final relevance score
  - source_type: string - "selected_text" or "database"
  - source: string - Original source identifier

## Validation Rules
- RAGResponse.response must not be empty when sources exist
- RAGResponse.sources must contain valid chunk_id, content, and score fields
- QueryContext.combined_context must be sorted by relevance score (descending)
- QueryContext must maintain minimum relevance threshold (0.1) for selected text chunks

## State Transitions
- QueryContext: INITIALIZED → PROCESSED (when selected text and database search are completed)
- QueryContext: PROCESSED → RANKED (when chunks are combined and ranked by relevance)
- RAGResponse: GENERATED → VALIDATED (when response quality and format are verified)