# Implementation Tasks: Chatbot Floating Widget

**Feature**: Chatbot Floating Widget
**Branch**: `001-chatbot-floating-widget`
**Created**: 2025-12-16
**Status**: Draft

## Implementation Strategy

This feature will transform the existing embedded chatbot into a floating widget button that remains closed by default. The implementation will follow an incremental approach, starting with the core functionality (User Story 1) and then adding responsive positioning (User Story 2) and visual consistency (User Story 3). Each user story will be independently testable and deliverable.

## Dependencies

User stories will be completed in priority order:
1. User Story 1 (P1) - Core floating widget functionality
2. User Story 2 (P2) - Responsive positioning
3. User Story 3 (P3) - Visual consistency

User Story 2 depends on User Story 1 completion. User Story 3 depends on User Story 1 completion.

## Parallel Execution Examples

Within each user story phase, the following tasks can be executed in parallel:
- CSS styling tasks with component logic tasks
- Different aspects of the same component (state management vs rendering)
- Testing tasks after implementation tasks are complete

## Phase 1: Setup

### Goal
Initialize project structure and ensure all necessary dependencies are in place for the floating widget implementation.

- [ ] T001 Set up development environment with required dependencies
- [ ] T002 Verify existing EmbeddedChatbot.jsx component is accessible
- [ ] T003 Verify existing embedded-chatbot.css is accessible
- [ ] T004 Verify backend API endpoint at http://localhost:8000/api/ask is working

## Phase 2: Foundational

### Goal
Prepare the base structure and state management needed for the floating widget functionality.

- [X] T005 [P] Modify EmbeddedChatbot.jsx to include floating widget state management
- [X] T006 [P] Update initial state in EmbeddedChatbot.jsx to closed by default
- [X] T007 [P] Add necessary CSS classes for floating button in embedded-chatbot.css
- [X] T008 [P] Create toggle functionality for widget open/close in EmbeddedChatbot.jsx
- [X] T009 [P] Ensure existing chat functionality remains intact in EmbeddedChatbot.jsx

## Phase 3: User Story 1 - Replace Loading Bar with Floating Widget Button (Priority: P1)

### Goal
Implement the core functionality to replace the current chatbot loading bar icon with a floating widget button that remains closed by default when the page loads or refreshes, and only opens when the user clicks on the floating button.

### Independent Test Criteria
The floating widget button should be visible on the page when loaded, remain closed by default, and open when clicked. The original loading bar icon should no longer appear.

- [X] T010 [US1] Implement fixed positioning for the chat widget container in embedded-chatbot.css
- [X] T011 [US1] Create circular floating button with chat icon in embedded-chatbot.css
- [X] T012 [US1] Implement button click handler to toggle widget visibility in EmbeddedChatbot.jsx
- [X] T013 [US1] Update component rendering logic to show floating button when closed in EmbeddedChatbot.jsx
- [X] T014 [US1] Ensure widget remains closed on page load/refresh in EmbeddedChatbot.jsx
- [X] T015 [US1] Implement close functionality for the chat interface in EmbeddedChatbot.jsx
- [X] T016 [US1] Add click outside to close functionality in EmbeddedChatbot.jsx
- [X] T017 [US1] Test that widget opens when button is clicked
- [X] T018 [US1] Test that widget closes when close button is clicked
- [X] T019 [US1] Test that widget closes when clicking outside the widget
- [X] T020 [US1] Verify existing chat functionality still works when widget is open

## Phase 4: User Story 2 - Persistent Widget Position (Priority: P2)

### Goal
Ensure the floating chatbot widget button maintains its position consistently across different screen sizes and page layouts, ensuring accessibility for users regardless of how they interact with the book content.

### Independent Test Criteria
The floating button should maintain its position and visibility when resizing the browser window, rotating mobile devices, or navigating between different pages in the book.

- [X] T021 [US2] Implement responsive positioning that works across different screen sizes in embedded-chatbot.css
- [X] T022 [US2] Add media queries for mobile device support in embedded-chatbot.css
- [X] T023 [US2] Test widget positioning on different screen sizes
- [X] T024 [US2] Implement scroll position preservation for the widget in EmbeddedChatbot.jsx
- [X] T025 [US2] Add z-index management to ensure widget stays on top in embedded-chatbot.css
- [X] T026 [US2] Test widget behavior during window resize events
- [X] T027 [US2] Test widget positioning on mobile devices
- [X] T028 [US2] Verify widget doesn't overlap with other UI elements on small screens

## Phase 5: User Story 3 - Visual Consistency (Priority: P3)

### Goal
Ensure the new floating widget button maintains visual harmony with the existing design of the humanoid robotics book website while clearly indicating its function as a chatbot interface.

### Independent Test Criteria
The floating widget button should match the color scheme and styling of the website while standing out enough to be noticed by users who need assistance.

- [X] T029 [US3] Update widget colors to match site theme in embedded-chatbot.css
- [X] T030 [US3] Implement consistent styling with the overall website design in embedded-chatbot.css
- [X] T031 [US3] Add hover and active states for the floating button in embedded-chatbot.css
- [X] T032 [US3] Ensure button is visually recognizable as an interactive element
- [X] T033 [US3] Test visual consistency across different pages of the book
- [X] T034 [US3] Add accessibility attributes (ARIA labels) to the widget in EmbeddedChatbot.jsx
- [X] T035 [US3] Verify keyboard navigation works with the floating widget in EmbeddedChatbot.jsx

## Phase 6: Polish & Cross-Cutting Concerns

### Goal
Complete the implementation with additional features, animations, and edge case handling to provide a polished user experience.

- [X] T036 Add slide in/out animations for widget open/close in embedded-chatbot.css
- [X] T037 Implement loading states for the floating widget in EmbeddedChatbot.jsx
- [X] T038 Add error handling for API failures in EmbeddedChatbot.jsx
- [X] T039 Implement keyboard shortcuts for widget control in EmbeddedChatbot.jsx
- [X] T040 Add focus management for accessibility in EmbeddedChatbot.jsx
- [X] T041 Handle edge case: widget overlapping with other UI elements on small screens
- [X] T042 Handle edge case: widget overlapping with other UI elements on small screens
- [X] T043 Handle edge case: preventing multiple widget instantiation in EmbeddedChatbot.jsx
- [X] T044 Add performance optimizations for smooth animations in embedded-chatbot.css
- [X] T045 Write comprehensive tests for the floating widget functionality
- [X] T046 Update documentation for the new widget behavior
- [ ] T047 Conduct final testing across browsers and devices
- [X] T048 Perform final review of all changes and ensure code quality