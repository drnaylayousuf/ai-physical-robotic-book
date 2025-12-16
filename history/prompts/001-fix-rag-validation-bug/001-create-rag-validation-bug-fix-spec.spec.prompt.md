---
id: 1
title: Create RAG validation bug fix spec
stage: spec
date_iso: 2025-12-13sp
surface: agent
model: claude-sonnet-4-5-20250929
feature: fix-rag-validation-bug
branch: 001-fix-rag-validation-bug
user: user
command: /sp.specify
labels: ["spec", "rag", "validation", "bug-fix"]
links:
  spec: "specs/001-fix-rag-validation-bug/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "specs/001-fix-rag-validation-bug/spec.md"
  - "specs/001-fix-rag-validation-bug/checklists/requirements.md"
tests_yaml: []
prompt_text: |
  I have a RAG backend using FastAPI, Qdrant, Cohere embeddings, and Gemini 2.5 Flash.

  There is a logic bug in my /api/ask endpoint.

  Problem:
  The backend accepts requests with:
  - mode = "selected_text"
  - selected_text = null or empty

  Even when selected_text is null, the system still runs the selected_text RAG path.
  This causes Gemini to receive empty or insufficient context and respond with:
  "The book does not provide details about this topic."

  This is not a retrieval failure — Qdrant returns chunks successfully — but a prompt/context restriction bug.

  Required fix:
  Add strict validation so that:
  - If mode == "selected_text" AND selected_text is null/empty:
    → automatically fallback to mode = "full_book"
    OR
    → return a 400 validation error

  Please update the backend logic to enforce this rule and prevent empty-context RAG calls.
  Also ensure logging clearly indicates when a fallback occurs.
response_text: |
  Created feature specification for fixing the RAG validation bug.
  - Created feature branch 001-fix-rag-validation-bug
  - Generated comprehensive spec with user scenarios, requirements, and success criteria
  - Created quality checklist to validate the specification
  - Specification addresses the core issue of empty selected_text being processed incorrectly
  - Includes fallback behavior options and proper logging requirements
outcome: |
  Successfully created feature specification for fixing the RAG validation bug.
  The spec defines clear requirements for handling empty selected_text values when mode is "selected_text",
  with options for either falling back to "full_book" mode or returning 400 validation errors.
  Quality checklist confirms all requirements are met and the spec is ready for planning.
evaluation: |
  The specification comprehensively addresses the user's requirements with clear acceptance
  scenarios, functional requirements, and success criteria. The spec is ready to proceed
  to the planning phase.
---