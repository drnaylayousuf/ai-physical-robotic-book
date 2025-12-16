# API Contract: Embedded Chatbot Integration

## Overview
This contract defines the API interactions for the embedded chatbot component in Docusaurus. The embedded chatbot maintains full compatibility with the existing backend API to ensure feature parity.

## Endpoints

### POST /api/ask
Ask a question to the RAG system with page context.

#### Request
```json
{
  "question": "string, the user's question",
  "mode": "enum, 'full_book' or 'selected_text'",
  "selected_text": "string, optional text selected by user",
  "page_context": {
    "page_id": "string, identifier for current page",
    "page_title": "string, title of current page",
    "page_content_preview": "string, optional content preview"
  },
  "user_id": "string, optional user identifier for authenticated requests"
}
```

#### Response
```json
{
  "response": "string, the generated answer",
  "sources": [
    {
      "chunk_id": "string, identifier for the source chunk",
      "content": "string, the source content snippet",
      "confidence": "float, confidence score 0-1",
      "page_reference": "string, page reference for the source"
    }
  ],
  "references": ["string, array of paragraph/chapter IDs"],
  "timestamp": "string, ISO date string"
}
```

#### Error Responses
- `400 Bad Request`: Invalid request format
- `401 Unauthorized`: Authentication required
- `429 Too Many Requests`: Rate limiting
- `500 Internal Server Error`: Backend processing error

### GET /api/health
Check the health status of the backend API.

#### Response
```json
{
  "status": "string, 'healthy' or 'unhealthy'",
  "timestamp": "string, ISO date string",
  "services": {
    "qdrant": "string, 'connected' or 'disconnected'",
    "gemini": "string, 'connected' or 'disconnected'",
    "database": "string, 'connected' or 'disconnected'"
  }
}
```

### GET /api/metadata
Get book metadata and chapter information.

#### Response
```json
{
  "book_title": "string, title of the book",
  "chapters": [
    {
      "id": "string, chapter identifier",
      "title": "string, chapter title",
      "path": "string, URL path for the chapter",
      "word_count": "integer, approximate word count",
      "summary": "string, brief chapter summary"
    }
  ],
  "last_updated": "string, ISO date string",
  "version": "string, book version"
}
```

## Component-to-Backend Communication

### Authentication Flow
The embedded chatbot follows the same authentication flow as the standalone version:
1. Check for existing authentication token
2. If none exists, direct user to login
3. Include token in all API requests

### Error Handling
- Network errors: Display user-friendly message and retry mechanism
- API errors: Log error and display appropriate message to user
- Timeout errors: Show timeout message and allow retry

## Frontend State Management Contract

### Message Format
```json
{
  "id": "string, unique identifier for the message",
  "sender": "enum, 'user' or 'assistant'",
  "content": "string, the message content",
  "timestamp": "string, ISO date string",
  "status": "enum, 'sent', 'pending', 'delivered', 'error'",
  "sources": ["object, source citations for assistant responses"]
}
```

### Component State
```json
{
  "isVisible": "boolean, whether chatbot panel is visible",
  "isMinimized": "boolean, whether chatbot panel is minimized",
  "messages": "array of message objects",
  "currentInput": "string, current text in input field",
  "isLoading": "boolean, whether waiting for response",
  "selectedText": "string, text currently selected on page",
  "userAuthStatus": "enum, 'authenticated', 'guest', 'checking'"
}
```

## Page Context Integration

### Context Extraction
The embedded chatbot should extract and send the following context from the current page:
- Page title
- Page URL/permalink
- Content preview (first 500 characters)
- Current section heading

### Text Selection Integration
When text is selected on the page:
1. Extract the selected text
2. Optionally include surrounding context
3. Send to backend with `mode: 'selected_text'`

## Security Considerations

### Input Validation
- Sanitize all user inputs before sending to backend
- Validate content length limits
- Prevent XSS by sanitizing responses before display

### Rate Limiting
- Implement client-side rate limiting to prevent spam
- Respect server-side rate limits
- Queue messages during rate limit periods

### Data Privacy
- Do not store user messages locally without consent
- Clear chat history on navigation when appropriate
- Use secure storage for authentication tokens