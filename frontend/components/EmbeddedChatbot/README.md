# Embedded Chatbot Component

## Overview
The Embedded Chatbot component provides a RAG-based chat interface that's integrated directly into Docusaurus book pages, allowing users to ask questions about the content while reading.

## Features
- **Context-Aware**: Uses page context to provide relevant answers
- **Text Selection**: Users can select text and ask questions about it
- **Responsive Design**: Adapts to different screen sizes (desktop, tablet, mobile)
- **Authentication Support**: Works with existing authentication system
- **Citation Display**: Shows sources for responses
- **Error Handling**: Graceful error handling with user feedback
- **Performance Monitoring**: Tracks API call performance

## State Management
- `isVisible`: Controls visibility of the chatbot panel
- `isMinimized`: Controls minimized state of the chatbot
- `messages`: Array of chat messages (user and assistant)
- `currentInput`: Current text in the input field
- `isLoading`: Loading state during API requests
- `selectedText`: Text currently selected on the page
- `userAuthStatus`: Authentication status ('guest', 'authenticated')

## API Integration
The component communicates with the backend via:
- `/api/ask` - For chat queries
- Supports both 'full_book' and 'selected_text' modes
- Includes page context in requests
- Handles authentication tokens

## Event Handling
- Text selection detection from the page
- Form submission for questions
- Minimize/expand functionality
- Proper cleanup of event listeners on unmount

## Performance Considerations
- API call timing is monitored and logged
- Efficient state updates
- Proper cleanup of resources
- Error boundaries to prevent crashes

## Responsive Design
- Adapts to different screen sizes using CSS media queries
- Touch-friendly controls for mobile devices
- Optimized layout for both desktop and mobile

## Accessibility
- Proper ARIA labels
- Keyboard navigation support
- Screen reader compatibility
- Focus management