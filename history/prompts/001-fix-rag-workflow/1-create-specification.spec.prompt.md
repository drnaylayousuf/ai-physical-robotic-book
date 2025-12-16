---
id: 1
title: create-specification
stage: spec
date: 2025-12-11
surface: agent
model: sonnet-4-5-20250929
feature: fix-rag-workflow
branch: 001-fix-rag-workflow
user: nayla
command: /sp.specify
labels: ["rag", "qdrant", "gemini", "ai", "backend"]
links:
  spec: ./spec.md
  ticket: null
  adr: null
  pr: null
files:
  - C:\Users\nayla\OneDrive\Desktop\humanoid-robotics-book\specs\001-fix-rag-workflow\spec.md
  - C:\Users\nayla\OneDrive\Desktop\humanoid-robotics-book\specs\001-fix-rag-workflow\checklists\requirements.md
tests: []
---

# Create Specification

## User Input

```
I have a FastAPI RAG backend using Qdrant. My problem is:

My chatbot only returns a placeholder response:
"Based on the provided context, here is the answer…"

sources: [] is empty → Qdrant is not returning chunks.

My frontend is sending "mode": "selected_text" instead of "full_book".

My RAG system is NOT calling an LLM (Gemini).

My generate_response() function is just a fake template.

I need you to rewrite and fix my entire RAG workflow with the following features:

1. Fix the retrieval layer

Please:

Check if the collection exists

Check if embeddings are stored

Fix the Qdrant retrieval logic

Ensure retrieved chunks return:

chunk_id

content

score/confidence

source metadata

2. Add REAL Gemini LLM generation

Replace this fake placeholder:

response = f"Based on the provided context…"


With a REAL Gemini call using:

from google import genai

client = genai.Client(api_key=settings.GEMINI_API_KEY)


Make the model prompt like this:

Use the following context to answer the question.
If the context does not contain enough information,
say "The book does not provide details about this topic."


Use gemini-1.5-pro or gemini-1.5-flash.

Return a clean, helpful, detailed answer.

3. Fix and improve my process_query()

Make sure:

It sends mode="full_book" by default

It returns:

{
  "response": "...",
  "sources": [...],
  "references": [...]
}


It properly logs:

errors

performance

chunk retrieval

Gemini LLM tokens

4. Fix selected_text mode

Make sure selected_text mode:

cleans the text

chunks the text

embeds the text

ranks chunks by similarity

returns them sorted

But still uses Gemini to generate the final answer.

5. Add a full diagnostic check for ingestion

I want a Python snippet that:

connects to Qdrant

prints collection names

prints number of chunks

prints first 3 chunks from the collection

6. Update my frontend call

Make sure the chatbot sends:

{
  "question": "USER QUESTION",
  "mode": "full_book"
}


Not "selected_text".

If possible, show me an example fetch() request:

fetch("http://localhost:8000/api/ask", {
  method: "POST",
  headers: { "Content-Type": "application/json" },
  body: JSON.stringify({
    question: userMessage,
    mode: "full_book"
  })
})

7. Produce:

✔ Clean, production-ready updated RAGModel class
✔ Updated generate_response() using Gemini
✔ Updated retrieve_context()
✔ Updated process_query()
✔ A diagnostic script to check ingestion
✔ Frontend fetch example

Make the code organized, clean, and fully working.
```

## Response Summary

Created comprehensive feature specification for fixing the RAG workflow with Qdrant and Gemini integration. The specification includes:

- Three prioritized user stories covering core functionality (P1: Real RAG responses, P2: Support both modes, P3: Diagnostic checks)
- Detailed acceptance scenarios for each user story
- Comprehensive functional requirements (FR-001 through FR-010) covering Qdrant integration, Gemini API usage, response format, and diagnostic functionality
- Identified key entities (Qdrant Collection, RAG Response, Retrieved Chunks)
- Measurable success criteria (SC-001 through SC-005) with specific metrics
- Edge cases covering various error conditions and boundary scenarios

The specification is complete, testable, and ready for the planning phase. A quality checklist was created and all items are marked as completed.

## Outcome

The specification is ready for planning and implementation. All requirements are clearly defined with measurable success criteria, enabling the development team to create a proper implementation plan.