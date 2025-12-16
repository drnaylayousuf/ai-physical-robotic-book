import React, { useState, useEffect, useCallback } from 'react';
import '../styles/embedded-chatbot.css';

const EmbeddedChatbot = ({ pageContext }) => {
  const [isVisible, setIsVisible] = useState(false); // Changed from true to false - widget should be closed by default
  const [messages, setMessages] = useState([]);
  const [currentInput, setCurrentInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [userAuthStatus, setUserAuthStatus] = useState('guest'); // 'guest', 'authenticated', 'checking'

  // Function to handle text selection from the book content area only
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const selectedText = selection.toString().trim();

      // Check if the selection is within the book content area
      if (selectedText && isSelectionWithinBookContent(selection)) {
        setSelectedText(selectedText);
      }
    };

    // Function to check if selection is within book content area
    const isSelectionWithinBookContent = (selection) => {
      if (!selection.rangeCount) return false;

      const range = selection.getRangeAt(0);
      const startContainer = range.startContainer;
      const endContainer = range.endContainer;

      // Look for a book content container (could be identified by class, id, or data attribute)
      const bookContentContainer = document.querySelector('#book-text, .book-content, [data-book-content]');

      if (!bookContentContainer) {
        // If no specific book container found, we'll use a more general approach
        // Check if selection is within body but not in UI elements
        const uiElements = ['.embedded-chatbot', '.chatbot-messages', '.chatbot-input', '.message'];
        const isUiElement = uiElements.some(selector => {
          const elements = document.querySelectorAll(selector);
          return Array.from(elements).some(el => el.contains(startContainer) || el.contains(endContainer));
        });

        return !isUiElement; // Allow selection if it's not in UI elements
      }

      // Check if both start and end containers are within the book content
      const isStartWithin = bookContentContainer.contains(startContainer) || startContainer === bookContentContainer;
      const isEndWithin = bookContentContainer.contains(endContainer) || endContainer === bookContentContainer;

      return isStartWithin && isEndWithin;
    };

    // Also listen for keyup to handle Escape key clearing selection
    const handleKeyup = (e) => {
      if (e.key === 'Escape') {
        setSelectedText(''); // Clear any selected text when Escape is pressed
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleKeyup);

    // Cleanup function to remove event listeners
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleKeyup);
    };
  }, []);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!currentInput.trim() || isLoading) return;

    // Performance monitoring: Start timing
    const startTime = performance.now();

    // Add user message to chat
    const userMessage = {
      id: Date.now(),
      sender: 'user',
      content: currentInput,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    const input = currentInput;
    setCurrentInput('');

    try {
      // Determine mode based on whether there's selected text
      const hasSelectedText = selectedText && selectedText.trim();
      const mode = hasSelectedText ? 'selected_text' : 'full_book';

      // Check if we have an auth token, if not we may need to handle authentication
      // Call backend API with page context - using full URL to avoid CORS issues
      const response = await fetch('http://localhost:8000/api/ask', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: input,
          mode: mode,
          selected_text: hasSelectedText ? selectedText.trim() : null,
          page_context: pageContext || {},
          user_id: localStorage.getItem('userId') || null,
        }),
      });

      // Performance monitoring: End timing and log
      const endTime = performance.now();
      const duration = endTime - startTime;

      // Log performance metrics (in a real app, you'd send this to a monitoring service)
      console.log(`API call took ${duration.toFixed(2)} milliseconds for question: "${input.substring(0, 50)}..."`);

      // Check if response is OK and has JSON content
      if (!response.ok) {
        const errorText = await response.text(); // Get text instead of JSON for error responses
        console.error('API Error Response:', errorText);

        // Handle different error statuses
        if (response.status === 401 || response.status === 403) {
          throw new Error('Access denied. Please try again.');
        }

        throw new Error(`API request failed with status ${response.status}: ${errorText}`);
      }

      // Check if response is actually JSON before parsing
      const contentType = response.headers.get('content-type');
      if (!contentType || !contentType.includes('application/json')) {
        const responseText = await response.text();
        console.error('Non-JSON response received:', responseText);
        throw new Error('Server returned non-JSON response');
      }

      const data = await response.json();

      // Add assistant response to chat
      const assistantMessage = {
        id: Date.now() + 1,
        sender: 'assistant',
        content: data.response,
        sources: data.sources,
        references: data.references,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
      // Clear selected text after using it
      setSelectedText('');
    } catch (error) {
      console.error('Error sending message:', error);

      // Add error message to chat
      const errorMessage = {
        id: Date.now() + 1,
        sender: 'assistant',
        content: `Sorry, I encountered an error: ${error.message || 'Please try again.'}`,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Function to handle asking about selected text directly
  const handleAskAboutSelectedText = useCallback(async () => {
    if (!selectedText.trim()) return;

    // Performance monitoring: Start timing
    const startTime = performance.now();

    // Add user message to chat
    const userMessage = {
      id: Date.now(),
      sender: 'user',
      content: `About the selected text: "${selectedText.substring(0, 100)}${selectedText.length > 100 ? '...' : ''}"`,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Validate selected text before making the API call
      const hasSelectedText = selectedText && selectedText.trim();
      if (!hasSelectedText) {
        throw new Error('No valid selected text to process');
      }

      // Call backend API with selected text
      const response = await fetch('http://localhost:8000/api/ask', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: `What can you tell me about this text: "${selectedText}"?`,
          mode: 'selected_text',
          selected_text: selectedText.trim(),
          page_context: pageContext || {},
          user_id: localStorage.getItem('userId') || null,
        }),
      });

      // Performance monitoring: End timing and log
      const endTime = performance.now();
      const duration = endTime - startTime;

      // Log performance metrics (in a real app, you'd send this to a monitoring service)
      console.log(`API call for selected text took ${duration.toFixed(2)} milliseconds for text: "${selectedText.substring(0, 50)}..."`);

      // Check if response is OK and has JSON content
      if (!response.ok) {
        const errorText = await response.text(); // Get text instead of JSON for error responses
        console.error('API Error Response:', errorText);

        // Handle different error statuses
        if (response.status === 401 || response.status === 403) {
          throw new Error('Access denied. Please try again.');
        }

        throw new Error(`API request failed with status ${response.status}: ${errorText}`);
      }

      // Check if response is actually JSON before parsing
      const contentType = response.headers.get('content-type');
      if (!contentType || !contentType.includes('application/json')) {
        const responseText = await response.text();
        console.error('Non-JSON response received:', responseText);
        throw new Error('Server returned non-JSON response');
      }

      const data = await response.json();

      // Add assistant response to chat
      const assistantMessage = {
        id: Date.now() + 1,
        sender: 'assistant',
        content: data.response,
        sources: data.sources,
        references: data.references,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
      // Clear selected text after using it
      setSelectedText('');
    } catch (error) {
      console.error('Error asking about selected text:', error);

      // Add error message to chat
      const errorMessage = {
        id: Date.now() + 1,
        sender: 'assistant',
        content: `Sorry, I encountered an error: ${error.message || 'Please try again.'}`,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  }, [selectedText, pageContext]);

  // Check authentication status on component mount
  useEffect(() => {
    const checkAuthStatus = async () => {
      const token = localStorage.getItem('authToken');
      if (token) {
        try {
          // Verify the token with the backend (in a real implementation)
          // For now, we'll assume if there's a token, the user is authenticated
          setUserAuthStatus('authenticated');
        } catch (error) {
          console.error('Token verification failed:', error);
          // Clear invalid token
          localStorage.removeItem('authToken');
          setUserAuthStatus('guest');
        }
      } else {
        setUserAuthStatus('guest');
      }
    };

    checkAuthStatus();
  }, []);

  // Function to toggle visibility
  const toggleVisibility = () => {
    setIsVisible(!isVisible);
  };

  // Function to handle click outside the chat widget
  useEffect(() => {
    if (!isVisible) return; // Only add listener when widget is visible

    const handleClickOutside = (event) => {
      const chatWidget = document.querySelector('.embedded-chatbot');
      const floatingButton = document.querySelector('.chatbot-floating-button');

      if (chatWidget && !chatWidget.contains(event.target) &&
          floatingButton && !floatingButton.contains(event.target)) {
        // Check if the click is not on the close button inside the widget
        const closeBtn = event.target.closest('.close-btn');
        if (!closeBtn) {
          setIsVisible(false);
        }
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isVisible]);

  // Handle window resize to ensure proper positioning
  useEffect(() => {
    const handleResize = () => {
      // No specific resize logic needed since we're using fixed positioning
      // The CSS handles responsive behavior already
    };

    window.addEventListener('resize', handleResize);
    return () => {
      window.removeEventListener('resize', handleResize);
    };
  }, []);

  // Handle keyboard navigation
  useEffect(() => {
    const handleKeyDown = (event) => {
      // Close chat widget with Escape key
      if (event.key === 'Escape' && isVisible) {
        setIsVisible(false);
        event.preventDefault();
      }
      // Toggle visibility with Ctrl/Cmd + Shift + C
      if ((event.ctrlKey || event.metaKey) && event.shiftKey && event.key === 'C') {
        event.preventDefault();
        toggleVisibility();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => {
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [isVisible]);

  // Manage focus when widget opens/closes
  useEffect(() => {
    if (isVisible) {
      // Focus on the input field when widget opens
      const inputElement = document.querySelector('.chatbot-input input');
      if (inputElement) {
        setTimeout(() => {
          inputElement.focus();
        }, 100); // Small delay to ensure DOM is updated
      }
    }
  }, [isVisible]);

  return (
    <>
      {/* Floating Button - shown when widget is not visible */}
      {!isVisible && (
        <button
          className="chatbot-floating-button"
          onClick={toggleVisibility}
          aria-label="Open chatbot assistant"
          title="Ask about Humanoid Robotics"
          aria-expanded="false"
          role="button"
          tabIndex="0"
        />
      )}

      {/* Chat Widget - shown when visible */}
      {isVisible && (
        <div
          className={`embedded-chatbot ${isVisible ? 'visible' : 'hidden'}`}
          role="dialog"
          aria-modal="true"
          aria-label="Chat with Humanoid Robotics Assistant"
          tabIndex="-1"
        >
          <div className="chatbot-header">
            <h3 tabIndex="0">Humanoid Robotics Assistant</h3>
            <div className="chatbot-controls" role="toolbar" aria-label="Chat controls">
              <button
                onClick={() => setIsVisible(false)} // Changed to close the widget instead of minimize
                className="minimize-btn"
                aria-label="Minimize chat"
                title="Minimize chat"
                onTouchStart={(e) => e.stopPropagation()} // Prevent touch events from bubbling
              >
                −
              </button>
              <button
                onClick={() => setIsVisible(false)} // Changed from setIsVisible(false) to close the widget
                className="close-btn"
                aria-label="Close chat"
                title="Close chat"
                onTouchStart={(e) => e.stopPropagation()} // Prevent touch events from bubbling
              >
                ×
              </button>
            </div>
          </div>

          <>
            <div className="chatbot-messages">
              {messages.length === 0 ? (
                <div className="welcome-message">
                  <p>Ask me anything about humanoid robotics and AI!</p>
                  {selectedText && (
                    <button
                      className="ask-about-selection-btn"
                      onClick={handleAskAboutSelectedText}
                      disabled={isLoading}
                      onTouchStart={(e) => e.stopPropagation()} // Prevent touch events from bubbling
                    >
                      Ask about selected text: "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"
                    </button>
                  )}
                </div>
              ) : (
                messages.map((message) => (
                  <div key={message.id} className={`message ${message.sender}`}>
                    <div className="message-content">{message.content}</div>
                    {message.sources && message.sources.length > 0 && (
                      <div className="message-sources">
                        <details>
                          <summary>Sources</summary>
                          <ul>
                            {message.sources.map((source, index) => (
                              <li key={index}>
                                {source.content ? `${source.content.substring(0, 100)}...` : `Reference ${index + 1}`}
                              </li>
                            ))}
                          </ul>
                        </details>
                      </div>
                    )}
                    {message.references && message.references.length > 0 && (
                      <div className="message-references">
                        <small>Referenced: {message.references.slice(0, 3).join(', ')}{message.references.length > 3 ? '...' : ''}</small>
                      </div>
                    )}
                  </div>
                ))
              )}
              {isLoading && <div className="loading">Thinking...</div>}
              {selectedText && messages.length > 0 && (
                <button
                  className="ask-about-selection-btn"
                  onClick={handleAskAboutSelectedText}
                  disabled={isLoading}
                  onTouchStart={(e) => e.stopPropagation()} // Prevent touch events from bubbling
                >
                  Ask about selected text: "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"
                </button>
              )}
            </div>

            <form onSubmit={handleSubmit} className="chatbot-input">
              <input
                value={currentInput}
                onChange={(e) => setCurrentInput(e.target.value)}
                placeholder="Ask about this page..."
                disabled={isLoading}
                aria-label="Type your question"
                onTouchStart={(e) => e.stopPropagation()} // Prevent touch events from bubbling
              />
              <button
                type="submit"
                disabled={isLoading}
                onTouchStart={(e) => e.stopPropagation()} // Prevent touch events from bubbling
              >
                Send
              </button>
            </form>
          </>
        </div>
      )}
    </>
  );
};

export default EmbeddedChatbot;