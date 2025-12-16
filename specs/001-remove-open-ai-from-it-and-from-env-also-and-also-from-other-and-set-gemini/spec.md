# Feature Specification: Remove OpenAI and Set Up Gemini

**Feature Branch**: `001-remove-open-ai-from-it-and-from-env-also-and-also-from-other-and-set-gemini`
**Created**: 2025-12-11
**Status**: Draft
**Input**: User description: "remove open ai  from it and from .env also  and also from other and set gemini"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Switch from OpenAI to Gemini for embeddings (Priority: P1)

As a user of the humanoid robotics book application, I want the system to use Google Gemini for embeddings instead of OpenAI, so that I can leverage the Gemini model for RAG (Retrieval-Augmented Generation) functionality.

**Why this priority**: This is the core functionality that needs to work for the application to function properly with the new AI provider.

**Independent Test**: Can be fully tested by running the application with Gemini configuration and verifying that embeddings are generated using Gemini instead of OpenAI, delivering the same quality of search and response functionality.

**Acceptance Scenarios**:

1. **Given** the application is configured with Gemini API key, **When** a user makes a query to the chatbot, **Then** the system uses Gemini embeddings to retrieve relevant context from the book
2. **Given** the application is configured with Gemini API key, **When** new content is ingested into the system, **Then** Gemini embeddings are generated and stored in the vector database

---

### User Story 2 - Remove OpenAI configuration and dependencies (Priority: P2)

As a developer maintaining the application, I want to remove all OpenAI-related configuration, code, and dependencies, so that the system is clean and only uses Gemini as the AI provider.

**Why this priority**: This is important for code maintenance and to avoid confusion with unused dependencies.

**Independent Test**: Can be fully tested by verifying that all OpenAI references are removed from codebase, dependencies, and configuration files, and the application still functions properly with Gemini.

**Acceptance Scenarios**:

1. **Given** the application codebase, **When** I search for OpenAI references, **Then** no unnecessary OpenAI code or configuration remains
2. **Given** the requirements file, **When** I check dependencies, **Then** OpenAI-related packages are removed from the project

---

### User Story 3 - Update environment configuration for Gemini (Priority: P3)

As a system administrator, I want to update the environment configuration to use Gemini instead of OpenAI, so that the application can connect to the correct AI service without errors.

**Why this priority**: This ensures the application can be properly configured in different environments.

**Independent Test**: Can be fully tested by verifying that the .env file contains only Gemini-related configuration and the application loads these settings correctly.

**Acceptance Scenarios**:

1. **Given** the .env file, **When** I check the configuration, **Then** it contains Gemini API key and appropriate settings
2. **Given** the application settings, **When** it starts up, **Then** it loads Gemini configuration without errors

---

### Edge Cases

- What happens when the Gemini API key is invalid or missing?
- How does the system handle rate limiting from the Gemini API?
- What happens if Gemini API returns an error during embedding generation?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST use Google Gemini API for generating embeddings instead of OpenAI
- **FR-002**: System MUST allow configuration of Gemini API key through environment variables
- **FR-003**: System MUST remove all OpenAI embedding functionality from the codebase
- **FR-004**: System MUST update the embedding service to use Gemini models for vector generation
- **FR-005**: System MUST continue to function properly with Qdrant vector database using Gemini embeddings
- **FR-006**: System MUST remove OpenAI API key configuration from environment settings
- **FR-007**: System MUST update the RAG (Retrieval-Augmented Generation) pipeline to use Gemini for context retrieval
- **FR-008**: System MUST maintain the same user experience while switching the underlying AI provider

### Key Entities

- **EmbeddingService**: Service responsible for generating vector embeddings from text, now using Gemini instead of OpenAI
- **Configuration**: Environment settings that include Gemini API key and model specifications instead of OpenAI equivalents

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Application successfully starts and runs with Gemini configuration instead of OpenAI
- **SC-002**: Embeddings are generated using Gemini API and stored in Qdrant vector database without errors
- **SC-003**: User queries to the chatbot return relevant responses using Gemini-powered RAG functionality
- **SC-004**: All OpenAI-related code, configuration, and dependencies are removed from the project
- **SC-005**: The application performance and response quality remains consistent or improves compared to OpenAI implementation
