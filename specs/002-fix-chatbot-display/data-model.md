# Data Model: Chatbot Display Fix

## Entity: Chat Response
- **Fields**:
  - response (string): The main response text from the AI model, may contain markdown formatting
  - sources (Array[Object]): List of source documents/chunks used for response generation
  - references (Array[string]): List of references cited in the response

- **Validation Rules**:
  - response: Required, should be properly formatted for display
  - sources: Optional, each source object has chunk_id, content, score, and source fields
  - references: Optional, array of reference identifiers

## Entity: Source Information
- **Fields**:
  - chunk_id (string): Unique identifier for the source chunk
  - content (string): The actual content from the source
  - score (number): Confidence/score value for the source (replaces deprecated 'confidence' field)
  - source (string): Identifier for the source collection

## Entity: Formatted Message
- **Fields**:
  - content (string or Object): Raw content before formatting
  - sender (string): "user" or "bot" indicating the message sender
  - formatted_content (string): HTML-formatted content ready for display

## State Transitions
- **Message Display Pipeline**:
  - Initial state: Raw API response received
  - Processing state: Markdown formatting applied, XSS protection implemented
  - Ready state: Formatted message ready for DOM insertion
  - Displayed state: Message rendered in UI

## Relationships
- Chat Response → Source Information (1:N, when sources exist)
- Formatted Message ← Chat Response (processed relationship)