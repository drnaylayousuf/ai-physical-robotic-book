# Feature Specification: Fix RAG Validation Bug

**Feature Branch**: `001-fix-rag-validation-bug`
**Created**: 2025-12-13
**Status**: Draft
**Input**: User description: "I have a RAG backend using FastAPI, Qdrant, Cohere embeddings, and Gemini 2.5 Flash.

There is a logic bug in my /api/ask endpoint.

Problem:
The backend accepts requests with:
- mode = \"selected_text\"
- selected_text = null or empty

Even when selected_text is null, the system still runs the selected_text RAG path.
This causes Gemini to receive empty or insufficient context and respond with:
\"The book does not provide details about this topic.\"

This is not a retrieval failure — Qdrant returns chunks successfully — but a prompt/context restriction bug.

Required fix:
Add strict validation so that:
- If mode == \"selected_text\" AND selected_text is null/empty:
  → automatically fallback to mode = \"full_book\"
  OR
  → return a 400 validation error

Please update the backend logic to enforce this rule and prevent empty-context RAG calls.
Also ensure logging clearly indicates when a fallback occurs."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Prevent Empty Context RAG Requests (Priority: P1)

When users submit RAG queries with selected_text mode but provide null or empty selected_text, the system should properly validate the input and either fallback to full book mode or return a validation error. This ensures that the AI model receives sufficient context for meaningful responses instead of returning "The book does not provide details about this topic."

**Why this priority**: This is a critical bug that leads to poor user experience with irrelevant responses. Without this fix, users will consistently receive unhelpful responses when they accidentally submit empty selections.

**Independent Test**: Can be fully tested by sending API requests with mode="selected_text" and empty/null selected_text, and verifying that the system either falls back to full_book mode or returns a 400 validation error.

**Acceptance Scenarios**:

1. **Given** a RAG API request with mode="selected_text" and selected_text=null, **When** the request is processed, **Then** the system should either fall back to full_book mode or return a 400 validation error
2. **Given** a RAG API request with mode="selected_text" and selected_text="", **When** the request is processed, **Then** the system should either fall back to full_book mode or return a 400 validation error
3. **Given** a RAG API request with mode="selected_text" and selected_text="valid text", **When** the request is processed, **Then** the system should proceed normally with selected_text mode

---

### User Story 2 - Clear Logging for Fallback Behavior (Priority: P2)

When the system detects an invalid selected_text request and applies the fallback logic, it should log this event clearly so that developers and operators can monitor when fallbacks occur and potentially identify patterns of improper usage.

**Why this priority**: Proper logging is essential for monitoring system behavior and identifying when users are submitting invalid requests, which can help improve the user interface or guide user education.

**Independent Test**: Can be tested by triggering fallback scenarios and verifying that appropriate log entries are created indicating the fallback occurred.

**Acceptance Scenarios**:

1. **Given** a RAG API request triggers a fallback from selected_text to full_book, **When** the request is processed, **Then** the system should log a clear message indicating the fallback occurred

---

### User Story 3 - Maintain Normal RAG Operations (Priority: P3)

Valid RAG requests should continue to work as expected without any changes to their behavior, ensuring backward compatibility for properly formed requests.

**Why this priority**: Maintaining existing functionality is crucial to avoid breaking changes for users who are already using the system correctly.

**Independent Test**: Can be tested by sending valid requests with different modes and verifying they continue to work as before.

**Acceptance Scenarios**:

1. **Given** a valid RAG API request with mode="full_book", **When** the request is processed, **Then** the system should behave exactly as before
2. **Given** a valid RAG API request with mode="selected_text" and valid selected_text, **When** the request is processed, **Then** the system should behave exactly as before

---

### Edge Cases

- What happens when selected_text contains only whitespace characters?
- How does the system handle requests where selected_text is an empty string versus null?
- What if selected_text is extremely short (e.g., 1-2 characters)?
- How should the system handle requests with unusual whitespace characters (tabs, newlines, etc.)?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST validate that when mode="selected_text", the selected_text parameter is not null or empty before proceeding with the selected_text RAG path
- **FR-002**: System MUST either automatically fallback to mode="full_book" OR return a 400 validation error when mode="selected_text" and selected_text is null or empty
- **FR-003**: System MUST maintain all existing functionality for valid RAG requests without any behavioral changes
- **FR-004**: System MUST log a clear message when falling back from selected_text to full_book mode due to validation failure
- **FR-005**: System MUST return appropriate error messages to clients when rejecting invalid requests with 400 status codes
- **FR-006**: System MUST treat empty strings, null values, and whitespace-only strings as invalid for selected_text parameter
- **FR-007**: System MUST process selected_text requests normally when the selected_text parameter contains valid non-empty content

### Key Entities *(include if feature involves data)*

- **RAG Request**: Represents a query to the Retrieval Augmented Generation system with parameters including mode and selected_text
- **Validation Result**: Represents the outcome of validating a RAG request, determining if it should proceed, fallback, or be rejected

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Zero instances of "The book does not provide details about this topic" responses due to empty context in selected_text mode within 30 days of deployment
- **SC-002**: 100% of invalid selected_text requests (null/empty) are either properly handled with fallback or rejected with 400 error within 30 days of deployment
- **SC-003**: No regression in valid RAG request processing - 99%+ of valid requests continue to receive meaningful responses
- **SC-004**: Clear logging events are generated when fallback behavior occurs, allowing operators to monitor and track these incidents
