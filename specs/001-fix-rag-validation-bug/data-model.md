# Data Model: RAG Validation Bug Fix

## Entity: RAG Request
- **Fields**:
  - question (str): The user's question to be answered
  - mode (str): The RAG mode ("full_book" or "selected_text")
  - selected_text (Optional[str]): Text selected by user for selected_text mode
  - user_id (Optional[str]): Identifier for the requesting user

- **Validation Rules**:
  - question: Required, 3-1000 characters, must contain alphanumeric characters
  - mode: Must be either "full_book" or "selected_text"
  - selected_text: Required when mode is "selected_text" and must not be empty/null/whitespace-only
  - user_id: Optional, for tracking purposes

## Entity: RAG Response
- **Fields**:
  - response (str): The generated response from the AI model
  - sources (List[Dict]): List of source documents/chunks used for response generation
  - references (List[str]): List of references cited in the response

## Entity: Validation Result
- **Fields**:
  - is_valid (bool): Whether the request passed validation
  - effective_mode (str): The mode that should be used after validation (may differ from requested mode)
  - error_message (Optional[str]): Error message if validation failed
  - fallback_occurred (bool): Whether a fallback from selected_text to full_book occurred

## State Transitions
- **RAG Request Processing**:
  - Initial state: Request received with original mode
  - Validation state: Request validated, effective mode determined
  - Processing state: Request processed with effective mode
  - Response state: Response generated and returned

## Relationships
- RAG Request → Validation Result (1:1)
- RAG Request → RAG Response (1:1 for successful requests)
- RAG Request → User Query (1:1 for successful requests, stored in database)