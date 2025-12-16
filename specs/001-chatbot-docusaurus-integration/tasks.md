# Implementation Tasks: Chatbot Docusaurus Integration

**Feature**: Chatbot Docusaurus Integration
**Branch**: `001-chatbot-docusaurus-integration`
**Spec**: [specs/001-chatbot-docusaurus-integration/spec.md](specs/001-chatbot-docusaurus-integration/spec.md)
**Plan**: [specs/001-chatbot-docusaurus-integration/plan.md](specs/001-chatbot-docusaurus-integration/plan.md)

## Implementation Strategy

Implement the chatbot integration in priority order of user stories, starting with the core functionality (User Story 1) to create an MVP, then adding the additional features (User Story 2), and finally the responsive design enhancements (User Story 3). Each user story is designed to be independently testable and deliver value.

## Dependencies

- **User Story 2** depends on core components from **User Story 1** (EmbeddedChatbot component)
- **User Story 3** depends on the basic component implementation from **User Story 1**
- All user stories depend on the foundational setup tasks in Phases 1-2

## Parallel Execution Examples

- T003-T005 can be executed in parallel (different component files)
- T006-T007 can be executed in parallel (styling and hooks)
- T015-T016 can be executed in parallel (different API integration tasks)

## Phase 1: Setup (Project Initialization)

### Goal
Initialize the project structure and dependencies needed for the embedded chatbot component.

- [x] T001 Create frontend directory structure (components, styles, hooks, utils)
- [ ] T002 Install required dependencies (react-icons, @docusaurus/core)
- [x] T003 Set up basic EmbeddedChatbot component file in frontend/components/EmbeddedChatbot.jsx

## Phase 2: Foundational (Blocking Prerequisites)

### Goal
Implement the core infrastructure needed for all user stories: API communication, text selection, and context extraction.

- [x] T004 Create useChatbotAPI hook in frontend/hooks/useChatbotAPI.js for API communication
- [x] T005 Create text selection utility in frontend/utils/textSelection.js
- [x] T006 Create context extractor utility in frontend/utils/contextExtractor.js
- [x] T007 Create embedded chatbot styling in frontend/styles/embedded-chatbot.css
- [x] T008 Verify existing backend API endpoints are accessible (/api/ask, /api/health)

## Phase 3: User Story 1 - Embed Chatbot Component in Docusaurus Theme (Priority: P1)

### Goal
Enable the core functionality of embedding the chatbot in Docusaurus pages so users can interact with it while reading.

### Independent Test Criteria
- Loading any book page shows the embedded chatbot interface
- User can type a question and receive a response with proper citations
- Chatbot maintains position and functionality across page navigation

### Implementation Tasks
- [x] T009 [US1] Implement basic EmbeddedChatbot component with state management (isVisible, messages, currentInput, isLoading)
- [x] T010 [US1] Add UI elements: header, message display area, input field, send button
- [x] T011 [US1] Implement message sending functionality to backend API
- [x] T012 [US1] Implement response display with citations
- [x] T013 [US1] Add minimize/expand functionality to the chatbot panel
- [x] T014 [US1] Integrate the EmbeddedChatbot component into Docusaurus theme Layout.js
- [x] T015 [US1] Implement page context extraction and inclusion in API requests
- [x] T016 [US1] Test basic functionality: load page → see chatbot → ask question → get response

## Phase 4: User Story 2 - Maintain Chatbot Functionality in Embedded Context (Priority: P1)

### Goal
Ensure all existing functionality (authentication, text selection, mode switching, citations) works in the embedded context.

### Independent Test Criteria
- Authentication works the same as the standalone interface
- Text selection and "ask about selected text" functionality works
- All response features (citations, sources) work identically to standalone version

### Implementation Tasks
- [x] T017 [US2] Implement authentication state integration in EmbeddedChatbot component
- [x] T018 [US2] Connect text selection utility to the chatbot component
- [x] T019 [US2] Implement "ask about selected text" mode in API communication
- [x] T020 [US2] Display citations and sources properly in embedded chat interface
- [x] T021 [US2] Add loading states and error handling for API requests
- [x] T022 [US2] Test feature parity: authentication, text selection, citations work identically to standalone

## Phase 5: User Story 3 - Responsive Design and Layout Integration (Priority: P2)

### Goal
Make the embedded chatbot responsive and integrate well with the page layout without disrupting the reading experience.

### Independent Test Criteria
- Chatbot layout adapts appropriately on mobile devices (320px width)
- Chatbot layout works well on desktop devices (up to 1920px width)
- Chatbot doesn't interfere with content readability on any device size

### Implementation Tasks
- [x] T023 [US3] Implement responsive design for chatbot component using CSS media queries
- [x] T024 [US3] Adjust chatbot positioning and sizing for mobile devices
- [x] T025 [US3] Optimize chatbot interface for touch interactions on mobile
- [x] T026 [US3] Implement layout adjustments to prevent interference with content readability
- [x] T027 [US3] Add accessibility features (keyboard navigation, screen reader support)
- [x] T028 [US3] Test responsive behavior across different screen sizes (320px to 1920px)

## Phase 6: Polish & Cross-Cutting Concerns

### Goal
Final touches to ensure production readiness: performance optimization, error handling, and quality assurance.

### Implementation Tasks
- [x] T029 Optimize page load performance to ensure increase <20% with embedded chatbot
- [x] T030 Implement proper error boundaries for the embedded chatbot component
- [x] T031 Add performance monitoring for chatbot API calls
- [x] T032 Implement proper cleanup of event listeners and resources on component unmount
- [x] T033 Update documentation for the embedded chatbot implementation
- [x] T034 Test backward compatibility with existing standalone chatbot interface
- [x] T035 Verify the embedded chatbot does not interfere with Docusaurus search and navigation functionality
- [x] T036 Final integration testing: all features work together seamlessly across different browsers (Chrome, Firefox, Safari, Edge)