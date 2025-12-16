# Quickstart: Chatbot Docusaurus Integration

## Prerequisites

- Node.js v18+ installed
- Docusaurus v3.x project set up
- Existing chatbot backend API running
- Basic knowledge of React and Docusaurus theme customization

## Setup Steps

### 1. Install Dependencies

```bash
npm install react-icons # For UI icons
npm install @docusaurus/core # If not already installed
```

### 2. Create the Embedded Chatbot Component

Create `frontend/components/EmbeddedChatbot.jsx`:

```jsx
import React, { useState, useEffect } from 'react';
import { EmbeddedChatbot } from './EmbeddedChatbot';

// This is a simplified example - actual implementation will be more complex
const EmbeddedChatbot = ({ pageContext }) => {
  const [isVisible, setIsVisible] = useState(true);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim()) return;

    // Add user message to chat
    const userMessage = {
      id: Date.now(),
      sender: 'user',
      content: inputValue,
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Call backend API with page context
      const response = await fetch('/api/ask', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          question: inputValue,
          pageContext: pageContext,
          mode: 'full_book' // or 'selected_text' if text is selected
        })
      });

      const data = await response.json();

      // Add assistant response to chat
      const assistantMessage = {
        id: Date.now() + 1,
        sender: 'assistant',
        content: data.response,
        sources: data.sources,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
    } finally {
      setIsLoading(false);
      setInputValue('');
    }
  };

  return (
    <div className={`embedded-chatbot ${isVisible ? 'visible' : 'hidden'}`}>
      <div className="chatbot-header">
        <h3>Humanoid Robotics Assistant</h3>
        <button onClick={() => setIsVisible(!isVisible)}>
          {isVisible ? 'Minimize' : 'Expand'}
        </button>
      </div>
      <div className="chatbot-messages">
        {messages.map(message => (
          <div key={message.id} className={`message ${message.sender}`}>
            {message.content}
          </div>
        ))}
        {isLoading && <div className="loading">Thinking...</div>}
      </div>
      <form onSubmit={handleSubmit} className="chatbot-input">
        <input
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask about this page..."
          disabled={isLoading}
        />
        <button type="submit" disabled={isLoading}>
          Send
        </button>
      </form>
    </div>
  );
};

export default EmbeddedChatbot;
```

### 3. Integrate with Docusaurus Theme

Modify the main layout in `src/theme/Layout.js` or create a custom layout:

```jsx
import React from 'react';
import Layout from '@theme-original/Layout';
import EmbeddedChatbot from '@site/frontend/components/EmbeddedChatbot';

export default function LayoutWrapper(props) {
  return (
    <Layout {...props}>
      {/* Render the main content */}
      {props.children}

      {/* Add the embedded chatbot */}
      <EmbeddedChatbot
        pageContext={{
          pageId: props.metadata?.permalink,
          pageTitle: props.metadata?.title,
          textContent: props.metadata?.description
        }}
      />
    </Layout>
  );
}
```

### 4. Add Styling

Create `frontend/styles/embedded-chatbot.css`:

```css
.embedded-chatbot {
  position: fixed;
  bottom: 20px;
  right: 20px;
  width: 350px;
  height: 500px;
  border: 1px solid #ddd;
  border-radius: 8px;
  box-shadow: 0 4px 12px rgba(0,0,0,0.15);
  display: flex;
  flex-direction: column;
  background: white;
  z-index: 1000;
}

.embedded-chatbot.hidden {
  display: none;
}

.chatbot-header {
  padding: 12px;
  background: #f8f9fa;
  border-bottom: 1px solid #eee;
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.chatbot-messages {
  flex: 1;
  overflow-y: auto;
  padding: 12px;
}

.message.user {
  text-align: right;
  margin-bottom: 10px;
}

.message.assistant {
  text-align: left;
  margin-bottom: 10px;
  background: #f0f8ff;
  padding: 8px;
  border-radius: 4px;
}

.chatbot-input {
  padding: 12px;
  border-top: 1px solid #eee;
  display: flex;
}

.chatbot-input input {
  flex: 1;
  padding: 8px;
  border: 1px solid #ddd;
  border-radius: 4px;
  margin-right: 8px;
}

.chatbot-input button {
  padding: 8px 16px;
  background: #0077cc;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
}

.chatbot-input button:disabled {
  background: #cccccc;
  cursor: not-allowed;
}
```

### 5. Add Text Selection Feature

Create `frontend/utils/textSelection.js`:

```javascript
export const getSelectedText = () => {
  const selection = window.getSelection();
  return selection.toString().trim();
};

export const addTextSelectionListener = (callback) => {
  const handleSelection = () => {
    const selectedText = getSelectedText();
    if (selectedText) {
      callback(selectedText);
    }
  };

  document.addEventListener('mouseup', handleSelection);
  document.addEventListener('keyup', (e) => {
    if (e.key === 'Escape') {
      window.getSelection().removeAllRanges();
    }
  });

  return () => {
    document.removeEventListener('mouseup', handleSelection);
  };
};
```

### 6. Test the Integration

1. Start your Docusaurus development server: `npm run start`
2. Navigate to any documentation page
3. Verify that the chatbot appears and is functional
4. Test sending messages and receiving responses
5. Verify that text selection works and can be sent to the chatbot

## API Compatibility

The embedded chatbot uses the same API endpoints as the standalone version:
- `POST /api/ask` for chat requests
- `POST /api/ingest` for content ingestion (admin only)
- `GET /api/health` for health checks
- `GET /api/metadata` for book metadata

## Configuration Options

The chatbot can be configured via environment variables or props:
- `CHATBOT_VISIBLE_BY_DEFAULT`: Whether chatbot is visible on page load
- `CHATBOT_POSITION`: 'bottom-right', 'bottom-left', 'sidebar', etc.
- `ENABLE_TEXT_SELECTION`: Whether text selection integration is enabled