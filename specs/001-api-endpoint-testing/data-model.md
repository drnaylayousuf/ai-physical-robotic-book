# Data Model: API Endpoint Testing After Qdrant Cloud Migration

## Entities

### Book Content
- **Description**: The humanoid robotics book information stored as vectors in Qdrant Cloud for retrieval-augmented generation
- **Fields**:
  - content_id: unique identifier for each content chunk
  - text: the actual book content text
  - embedding: vector representation of the text
  - metadata: additional information about the content (section, chapter, etc.)
- **Relationships**: None direct, referenced by Qdrant Collections

### Qdrant Collections
- **Description**: Vector database collections containing the book content with proper indexing for semantic search
- **Fields**:
  - collection_name: name of the Qdrant collection
  - vector_size: dimension of the vectors stored
  - indexed_fields: fields that are indexed for search
  - total_vectors: count of vectors in the collection
- **Relationships**: Contains multiple Book Content entities

### Chat Requests
- **Description**: User queries containing questions and mode parameters that trigger RAG processing
- **Fields**:
  - question: the user's question text
  - mode: processing mode (e.g., "full_book")
  - timestamp: when the request was made
  - user_id: identifier for the requesting user (optional)
- **Relationships**: Triggers retrieval from Qdrant Collections

### Generated Responses
- **Description**: Answers created by Gemini model based on retrieved book content from Qdrant Cloud
- **Fields**:
  - response_text: the generated answer text
  - sources: references to the book content used to generate the response
  - confidence_score: confidence level of the response
  - timestamp: when the response was generated
- **Relationships**: Created from Chat Requests and Book Content

## Validation Rules

### From Functional Requirements
- FR-001: Health endpoint must confirm Qdrant Cloud connectivity
- FR-002: Diagnostic endpoint must report collection status and vector information
- FR-003: Chat endpoint must accept question and mode parameters
- FR-004: System must retrieve relevant book content from Qdrant Cloud
- FR-005: System must use Gemini model for response generation
- FR-006: System must support "full_book" mode
- FR-007: System must return appropriate error messages
- FR-008: System must be accessible via frontend applications

## State Transitions

### Chat Processing Flow
1. Chat Request received → Parameters validated
2. Parameters validated → Content retrieved from Qdrant Collections
3. Content retrieved → Gemini model generates response
4. Response generated → Response returned to client