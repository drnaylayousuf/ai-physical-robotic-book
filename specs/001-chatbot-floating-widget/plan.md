# Implementation Plan: Chatbot Floating Widget

**Branch**: `001-chatbot-floating-widget` | **Date**: 2025-12-16 | **Spec**: [link to spec](./spec.md)
**Input**: Feature specification from `/specs/001-chatbot-floating-widget/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Replace the current embedded chatbot loading bar icon with a floating widget button that remains closed by default when the page loads or refreshes. The widget should only open when the user clicks the floating button. The current implementation is a React component that appears as a persistent sidebar, but needs to be changed to a floating button that toggles visibility.

## Technical Context

**Language/Version**: JavaScript/ES2021, React 18.2.0
**Primary Dependencies**: React.js, CSS for styling, fetch API for backend communication
**Storage**: N/A (client-side state management with React hooks)
**Testing**: Jest for unit testing, React Testing Library for component testing
**Target Platform**: Web browser (Docusaurus documentation site)
**Project Type**: Web application with frontend and backend components
**Performance Goals**: <200ms response time for UI interactions, <500ms for API calls
**Constraints**: Must work across different screen sizes, maintain accessibility, integrate with existing Docusaurus layout
**Scale/Scope**: Single-page application with chatbot functionality across all book pages

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the constitution, this plan aligns with:
- Decision Point Mapping: Clear technical decisions identified for UI component transformation
- Reasoning Activation: Implementation requires understanding of React state management and CSS positioning
- Intelligence Accumulation: Reuses existing chatbot functionality while improving UX
- Right Altitude: Provides concrete technical approach without being overly prescriptive
- Frameworks Over Rules: Uses established React patterns for component state management
- Meta-Awareness Against Convergence: Avoids generic chatbot implementations, focusing on specific Docusaurus integration

## Project Structure

### Documentation (this feature)

```text
specs/001-chatbot-floating-widget/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
frontend/
├── components/
│   └── EmbeddedChatbot.jsx      # Modified to support floating widget behavior
├── styles/
│   └── embedded-chatbot.css     # Updated styles for floating widget
└── services/
    └── contextExtractor.js      # Page context extraction utility

src/
├── theme/
│   └── Layout.js               # Integration point for chatbot component
└── components/
    └── ErrorBoundary.js        # Error handling wrapper

backend/
└── api/
    └── ask                     # Backend API endpoint for chat functionality
```

**Structure Decision**: Web application with frontend React components and backend API services. The chatbot component will be modified to implement floating widget behavior while maintaining all existing functionality.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [None identified] | [N/A] | [N/A] |
