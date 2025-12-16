# Data Model: RAG System Error Handling

## RAGRequest Entity

### Attributes
- **question**: String representing the user's original question
- **mode**: String indicating the query mode ("full_book" or "selected_text")
- **selected_text**: Optional string for selected-text mode queries (can be null)
- **timestamp**: ISO 8601 datetime string of when the request was made

### Relationships
- **RAGResponse**: Each request generates one response
- **ErrorResponse**: Each request may result in an error response if processing fails

### Validation Rules
- Question must not be empty or contain only whitespace
- Mode must be either "full_book" or "selected_text"
- If mode is "selected_text", selected_text can be provided or null

## RAGResponse Entity

### Attributes
- **response**: String representing the generated answer to the user's question
- **sources**: Array of source objects containing retrieved context chunks
- **references**: Array of reference strings for citations

### Relationships
- **Sources**: Each RAG Response contains multiple source chunks retrieved from vector database
- **User Request**: Each response is generated based on a specific user request

### Validation Rules
- Response must not be empty when no error occurs
- Sources array may be empty but must be present
- Either response must be present OR error must be present in the final output

## ErrorResponse Entity

### Attributes
- **detail**: String containing human-readable error message
- **error_code**: Optional string containing machine-readable error code
- **timestamp**: ISO 8601 datetime string of when the error occurred
- **request_id**: Optional string for request correlation and debugging

### Relationships
- **RAGRequest**: Each error response corresponds to a specific request
- **DependencyStatus**: Error may relate to dependency unavailability

### Validation Rules
- Detail must be present and non-empty
- Timestamp must be in ISO 8601 format
- Should not expose internal system details to end users

## Retrieved Chunk Entity

### Attributes
- **chunk_id**: String identifier for the specific content chunk
- **content**: String containing the actual text content of the chunk
- **score**: Float representing the relevance score/confidence (0.0 to 1.0)
- **source**: String containing metadata about the original source document

### Relationships
- **Vector Database Collection**: Each chunk is stored in and retrieved from a vector database collection
- **RAG Response**: Each chunk may be included in a response's sources array

### Validation Rules
- Chunk_id must be unique within the collection
- Content must not be empty
- Score must be between 0.0 and 1.0
- Source metadata must be properly formatted

## DependencyStatus Entity

### Attributes
- **service_name**: String representing the service name ("vector_database", "embedding_service")
- **is_available**: Boolean indicating whether the service is accessible
- **details**: Optional string with additional information about service status
- **last_checked**: ISO 8601 datetime string of when the status was last verified

### Relationships
- **RAGRequest**: Status may be checked during request processing
- **ErrorResponse**: Unavailable dependencies may result in errors

### Validation Rules
- Service_name must be one of the predefined service types
- Is_available must be a boolean value
- Last_checked must be in ISO 8601 format

## State Transitions

### RAGRequest Processing Flow:
1. **Received**: Request received by API, parameters validated
2. **Validating Dependencies**: Service availability checked
3. **Processing**: Request being handled by RAG system
4. **Completed/Error**: Response generated or error occurred

### Dependency Status Flow:
1. **Unknown**: Initial state when service status is not yet checked
2. **Checking**: Currently validating service availability
3. **Available**: Service is accessible and responding
4. **Unavailable**: Service is not accessible or responding with errors