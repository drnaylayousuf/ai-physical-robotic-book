import { useState } from 'react';

const useChatbotAPI = () => {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  const askQuestion = async (question, mode = 'full_book', selectedText = null, pageContext = null) => {
    setIsLoading(true);
    setError(null);

    try {
      // Validate selected text and determine appropriate mode
      const hasSelectedText = selectedText && selectedText.trim();
      const effectiveMode = hasSelectedText ? 'selected_text' : 'full_book';

      const requestBody = {
        question,
        mode: effectiveMode,
      };

      if (hasSelectedText) {
        requestBody.selected_text = selectedText.trim();
      }

      if (pageContext) {
        requestBody.page_context = pageContext;
      }

      const response = await fetch('http://localhost:8000/api/ask', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (err) {
      console.error('Error in askQuestion:', err);
      setError(err.message);
      throw err;
    } finally {
      setIsLoading(false);
    }
  };

  const checkHealth = async () => {
    try {
      const response = await fetch('http://localhost:8000/api/health');
      const data = await response.json();
      return data;
    } catch (err) {
      console.error('Error checking health:', err);
      setError(err.message);
      throw err;
    }
  };

  const getMetadata = async () => {
    try {
      const response = await fetch('http://localhost:8000/api/metadata');
      const data = await response.json();
      return data;
    } catch (err) {
      console.error('Error getting metadata:', err);
      setError(err.message);
      throw err;
    }
  };

  return {
    askQuestion,
    checkHealth,
    getMetadata,
    isLoading,
    error,
  };
};

export default useChatbotAPI;