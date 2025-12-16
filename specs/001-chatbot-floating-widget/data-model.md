# Data Model: Chatbot Floating Widget

## Entity: ChatWidgetState

**Description**: Represents the state of the chat widget interface

**Attributes**:
- `isVisible` (boolean): Whether the chat widget is currently visible/open
- `position` (string): Current position state - either "floating" or "expanded"
- `lastInteractionTime` (timestamp): Time of last user interaction
- `messageHistory` (array): Collection of chat messages in the current session
- `isLoading` (boolean): Whether the widget is currently processing a request

**State Transitions**:
1. Initial State: `{isVisible: false, position: "floating"}`
2. On Button Click: `{isVisible: true, position: "expanded"}`
3. On Close/Minimize: `{isVisible: false, position: "floating"}`
4. On Message Send: `{isLoading: true}` → `{isLoading: false}`

**Validation Rules**:
- `isVisible` must be a boolean value
- `position` must be one of ["floating", "expanded"]
- `messageHistory` items must have `sender` and `content` properties

## Entity: FloatingButtonConfig

**Description**: Configuration for the floating button appearance and behavior

**Attributes**:
- `positionX` (string): Horizontal position (e.g., "right", "left")
- `positionY` (string): Vertical position (e.g., "bottom", "top")
- `offsetX` (string): Horizontal offset from edge (e.g., "20px")
- `offsetY` (string): Vertical offset from edge (e.g., "20px")
- `size` (string): Button size (e.g., "60px")
- `backgroundColor` (string): Button background color
- `iconColor` (string): Color of the chat icon

**Validation Rules**:
- Position values must be valid CSS position values
- Offset values must be valid CSS length units
- Size must be a valid CSS dimension

## Entity: ChatMessage

**Description**: Represents a single message in the chat conversation

**Attributes**:
- `id` (string): Unique identifier for the message
- `sender` (string): Either "user" or "assistant"
- `content` (string): Text content of the message
- `timestamp` (timestamp): When the message was sent/received
- `status` (string): Message status - "sending", "sent", "delivered", "error"

**Validation Rules**:
- `sender` must be either "user" or "assistant"
- `content` must be non-empty string
- `timestamp` must be valid ISO date string
- `status` must be one of the defined values

## Entity: ChatSession

**Description**: Represents a single chat session associated with the widget

**Attributes**:
- `sessionId` (string): Unique identifier for the session
- `startTime` (timestamp): When the session started
- `isActive` (boolean): Whether the session is currently active
- `pageContext` (object): Context information about the current page
- `messageCount` (number): Number of messages in the session

**Validation Rules**:
- `sessionId` must be unique
- `startTime` must be valid timestamp
- `messageCount` must be non-negative integer