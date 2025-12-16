# Data Model: Chatbot Docusaurus Integration

## EmbeddedChatbotComponent

### Fields
- `id` (string): Unique identifier for the component instance
- `isVisible` (boolean): Whether the chatbot panel is currently visible
- `isMinimized` (boolean): Whether the chatbot panel is minimized
- `messages` (array): List of chat messages in the conversation
- `currentInput` (string): Current text in the input field
- `isLoading` (boolean): Whether the component is waiting for a response
- `selectedText` (string): Text currently selected on the page
- `pageContext` (object): Information about the current page content
- `userAuthStatus` (string): Current authentication status

### Relationships
- `messages` contains `Message` objects
- `pageContext` references the current Docusaurus page content
- Related to global user authentication state

### Validation Rules
- `id` must be unique per page instance
- `messages` length should not exceed 100 items without archiving
- `currentInput` length should not exceed 2000 characters

### State Transitions
- `hidden` → `visible` when user activates the chatbot
- `visible` → `minimized` when user minimizes the panel
- `minimized` → `visible` when user expands the panel
- `idle` → `loading` when message is sent
- `loading` → `idle` when response is received

## Message

### Fields
- `id` (string): Unique identifier for the message
- `sender` (string): 'user' or 'assistant'
- `content` (string): The message content
- `timestamp` (date): When the message was created
- `sources` (array): Citations for assistant responses
- `pageRef` (string): Reference to the page where message was created

### Relationships
- Belongs to an `EmbeddedChatbotComponent`
- May reference `pageContext` for context-aware responses

### Validation Rules
- `sender` must be either 'user' or 'assistant'
- `content` must not be empty
- `timestamp` must be in the past or present

## PageContext

### Fields
- `pageId` (string): Unique identifier for the current page
- `pageTitle` (string): Title of the current page
- `textContent` (string): Extracted text content of the page
- `sectionInfo` (object): Information about current section
- `lastUpdated` (date): When the context was last updated

### Relationships
- Belongs to a Docusaurus page
- Referenced by `EmbeddedChatbotComponent`

### Validation Rules
- `pageId` must match the current Docusaurus page
- `textContent` should not exceed 100KB to avoid performance issues