# Tasks: Fix RAG Validation Bug

**Feature**: Fix RAG Validation Bug
**Branch**: `001-fix-rag-validation-bug`
**Date**: 2025-12-13
**Input**: Implementation plan from `/specs/001-fix-rag-validation-bug/plan.md`

## Implementation Strategy

This implementation follows an incremental delivery approach focusing on the critical bug fix first. The primary goal is to fix the validation bug where requests with mode="selected_text" and null/empty selected_text are incorrectly processed, causing Gemini to receive insufficient context. The solution maintains the existing fallback approach while fixing the underlying bug.

**MVP Scope**: User Story 1 (Prevent Empty Context RAG Requests) - This provides the core value by fixing the main bug.

## Dependencies

- User Story 1 (P1) must be completed before User Story 2 (P2) and User Story 3 (P3)
- No external dependencies beyond existing project infrastructure

## Parallel Execution Examples

- API layer fix (backend/api/chat.py) and RAG model fix (backend/models/rag.py) can be developed in parallel
- Unit tests can be written in parallel with implementation work
- Logging improvements can be implemented in parallel with validation fixes

---

## Phase 1: Setup

- [ ] T001 Set up development environment and verify existing tests pass
- [X] T002 Create backup of current implementation files before making changes

## Phase 2: Foundational Tasks

- [X] T003 [P] Review existing validation logic in backend/api/chat.py and backend/models/rag.py
- [ ] T004 [P] Create test cases to reproduce the RAG validation bug
- [ ] T005 [P] Document current behavior with empty selected_text in selected_text mode

## Phase 3: [US1] Prevent Empty Context RAG Requests (Priority: P1)

**Goal**: When users submit RAG queries with selected_text mode but provide null or empty selected_text, the system should properly validate the input and fallback to full book mode, ensuring that the AI model receives sufficient context for meaningful responses.

**Independent Test**: Can be fully tested by sending API requests with mode="selected_text" and empty/null selected_text, and verifying that the system falls back to full_book mode.

**Acceptance Scenarios**:
1. **Given** a RAG API request with mode="selected_text" and selected_text=null, **When** the request is processed, **Then** the system should fall back to full_book mode
2. **Given** a RAG API request with mode="selected_text" and selected_text="", **When** the request is processed, **Then** the system should fall back to full_book mode
3. **Given** a RAG API request with mode="selected_text" and selected_text="valid text", **When** the request is processed, **Then** the system should proceed normally with selected_text mode

- [X] T006 [P] [US1] Fix API layer validation in backend/api/chat.py to properly handle empty selected_text
- [X] T007 [P] [US1] Fix RAG model validation in backend/models/rag.py to prevent processing empty text in selected_text mode
- [X] T008 [US1] Update effective_mode handling to ensure empty selected_text is not passed to RAG model when falling back
- [X] T009 [US1] Test the fix with null selected_text in selected_text mode
- [X] T010 [US1] Test the fix with empty string selected_text in selected_text mode
- [X] T011 [US1] Test the fix with whitespace-only selected_text in selected_text mode
- [X] T012 [US1] Verify valid selected_text requests still work properly

## Phase 4: [US2] Clear Logging for Fallback Behavior (Priority: P2)

**Goal**: When the system detects an invalid selected_text request and applies the fallback logic, it should log this event clearly so that developers and operators can monitor when fallbacks occur.

**Independent Test**: Can be tested by triggering fallback scenarios and verifying that appropriate log entries are created indicating the fallback occurred.

**Acceptance Scenarios**:
1. **Given** a RAG API request triggers a fallback from selected_text to full_book, **When** the request is processed, **Then** the system should log a clear message indicating the fallback occurred

- [X] T013 [P] [US2] Add INFO level logging when fallback from selected_text to full_book occurs in backend/api/chat.py
- [X] T014 [P] [US2] Add INFO level logging when fallback from selected_text to full_book occurs in backend/models/rag.py
- [X] T015 [US2] Test that fallbacks generate appropriate log messages
- [X] T016 [US2] Verify log messages clearly indicate the reason for fallback

## Phase 5: [US3] Maintain Normal RAG Operations (Priority: P3)

**Goal**: Valid RAG requests should continue to work as expected without any changes to their behavior, ensuring backward compatibility for properly formed requests.

**Independent Test**: Can be tested by sending valid requests with different modes and verifying they continue to work as before.

**Acceptance Scenarios**:
1. **Given** a valid RAG API request with mode="full_book", **When** the request is processed, **Then** the system should behave exactly as before
2. **Given** a valid RAG API request with mode="selected_text" and valid selected_text, **When** the request is processed, **Then** the system should behave exactly as before

- [X] T017 [P] [US3] Test that full_book mode requests continue to work as before
- [X] T018 [P] [US3] Test that valid selected_text mode requests continue to work as before
- [X] T019 [US3] Run full test suite to ensure no regressions
- [X] T020 [US3] Verify performance metrics remain consistent

## Phase 6: Polish & Cross-Cutting Concerns

- [X] T021 Update documentation to reflect the new validation behavior
- [X] T022 Clean up any temporary test files or debug code
- [X] T023 Run complete test suite to ensure all functionality works as expected
- [X] T024 Perform final validation of all user stories
- [X] T025 Commit all changes with appropriate commit message