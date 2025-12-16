# Data Model: Chatbot UI/CSS Enhancement

## UI Components

### Chatbot Container
- **Component**: `.embedded-chatbot`
- **Properties**:
  - visibility (boolean)
  - minimized (boolean)
  - position (fixed bottom-right)
  - dimensions (width, height with responsive adjustments)

### Header Component
- **Component**: `.chatbot-header`
- **Properties**:
  - title: "Humanoid Robotics Assistant"
  - control buttons (minimize, close)
  - background styling

### Message Components
- **Component**: `.message` (user and assistant variants)
- **Properties**:
  - sender (user/assistant)
  - content (text)
  - timestamp
  - styling (background, color, alignment based on sender)

### Input Component
- **Component**: `.chatbot-input`
- **Properties**:
  - text input field
  - send button
  - disabled state during loading

### Sources Component
- **Component**: `.message-sources`
- **Properties**:
  - source references
  - expandable details

## Styling Variables (to be implemented)

### Color Palette
- Primary color: Enhanced blue for user messages and interactive elements
- Assistant message background: Enhanced light background
- Header background: Refined gradient or solid color
- Border colors: Improved contrast

### Typography
- Font stack: Enhanced with better fallbacks
- Font sizes: Refined for better readability
- Line heights: Optimized for content density

### Spacing & Layout
- Message padding: Improved for better readability
- Gap spacing: Consistent throughout the interface
- Border radius: Refined for modern appearance

## Responsive Breakpoints
- Desktop: 350px width, 500px height
- Tablet: 320px width, 450px height
- Mobile: Full width with max-width constraints
- Small screens: Optimized for limited real estate