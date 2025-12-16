# Chatbot Floating Widget Documentation

## Overview
The chatbot floating widget replaces the previous persistent sidebar implementation with a more user-friendly floating button that remains closed by default. The widget only opens when the user clicks the floating button, providing a less intrusive experience while maintaining all chat functionality.

## Key Features

### 1. Floating Button Design
- Circular button with chat icon appears in the bottom-right corner
- Subtle floating animation to draw attention without being distracting
- Proper positioning that respects safe areas on mobile devices

### 2. Default Closed State
- Widget starts in closed state when page loads or refreshes
- Floating button remains visible at all times for easy access
- Does not interfere with page content initially

### 3. Toggle Functionality
- Click floating button to open chat interface
- Click close (×) button or outside widget to close
- Keyboard shortcut: Press `Escape` to close when open

### 4. Responsive Design
- Adapts to different screen sizes
- Proper positioning on mobile devices
- Avoids overlapping with other UI elements

### 5. Accessibility Features
- Full keyboard navigation support
- ARIA labels for screen readers
- Proper focus management
- Keyboard shortcuts (Escape, Ctrl/Cmd+Shift+C)

## Usage

### Opening the Widget
1. Click the floating chat button in the bottom-right corner
2. The chat interface will slide in with animation

### Closing the Widget
1. Click the close (×) button in the header
2. Click outside the chat widget area
3. Press the `Escape` key on your keyboard

### Keyboard Shortcuts
- `Escape`: Close the widget when open
- `Ctrl+Shift+C` (or `Cmd+Shift+C` on Mac): Toggle widget visibility

## Technical Details

### State Management
- `isVisible`: Controls whether the widget is open (true) or closed (false)
- Default state is `false` (closed)
- State is managed using React's `useState` hook

### CSS Classes
- `.chatbot-floating-button`: Styles for the floating button
- `.embedded-chatbot.visible`: Styles when widget is open
- `.embedded-chatbot.hidden`: Styles when widget is closed

### Responsive Behavior
- On screens < 768px: Button size reduced to 55px
- On screens < 480px: Button size reduced to 50px
- On screens < 360px: Button size reduced to 46px
- On very small screens: Additional bottom margin to avoid UI overlap

## API Integration
The widget maintains all existing API integration functionality:
- Backend API endpoint: `http://localhost:8000/api/ask`
- Page context extraction remains unchanged
- Message history and state management preserved

## Browser Support
- Chrome, Firefox, Safari, Edge (latest versions)
- Mobile browsers on iOS and Android
- Responsive down to 320px screen width

## Accessibility Compliance
- Proper ARIA labels and roles
- Keyboard navigation support
- Screen reader compatible
- Focus management
- Reduced motion support via `prefers-reduced-motion` media query

## Known Limitations
- JavaScript must be enabled for full functionality
- Requires connection to backend API for chat responses
- Positioning may vary slightly across different Docusaurus themes