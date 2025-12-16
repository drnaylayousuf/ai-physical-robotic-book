# Quickstart: Chatbot Floating Widget

## Overview
This guide provides instructions for setting up and running the chatbot floating widget feature in the humanoid robotics book website.

## Prerequisites
- Node.js 18+ installed
- npm or yarn package manager
- Access to the backend API at `http://localhost:8000/api/ask`
- Docusaurus development environment

## Installation

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd humanoid-robotics-book
   ```

2. **Install dependencies**
   ```bash
   npm install
   # or
   yarn install
   ```

3. **Start the development server**
   ```bash
   npm run start
   # or
   yarn start
   ```

## Configuration

The floating widget is configured through the `EmbeddedChatbot.jsx` component. Key configuration options include:

- **Initial State**: Widget starts closed by default (controlled by React state)
- **Positioning**: Fixed position at bottom-right of screen
- **Styling**: Customizable through CSS variables in `embedded-chatbot.css`

## Development

To modify the floating widget behavior:

1. **Component Location**: `frontend/components/EmbeddedChatbot.jsx`
2. **Styles**: `frontend/styles/embedded-chatbot.css`
3. **Integration**: `src/theme/Layout.js`

## Testing

1. **Start the frontend**: `npm run start`
2. **Verify the floating button appears** on the bottom-right of the page
3. **Confirm the widget is closed by default** when the page loads
4. **Test the toggle functionality** by clicking the floating button
5. **Verify all chat functionality** works as expected when the widget is open

## API Integration

The chatbot communicates with the backend API at `http://localhost:8000/api/ask` for processing user queries. The API uses RAG (Retrieval Augmented Generation) to provide context-aware responses based on the humanoid robotics book content.

## Troubleshooting

- **Widget not appearing**: Check browser console for JavaScript errors
- **API calls failing**: Verify backend server is running at `http://localhost:8000`
- **Positioning issues**: Adjust CSS variables in `embedded-chatbot.css`
- **State not persisting**: Check React state management in `EmbeddedChatbot.jsx`