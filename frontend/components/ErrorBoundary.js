import React, { Component } from 'react';

class ErrorBoundary extends Component {
  constructor(props) {
    super(props);
    this.state = { hasError: false, error: null, errorInfo: null };
  }

  static getDerivedStateFromError(error) {
    // Update state so the next render will show the fallback UI
    return { hasError: true };
  }

  componentDidCatch(error, errorInfo) {
    // Log the error to an error reporting service
    console.error('EmbeddedChatbot error:', error, errorInfo);
    this.setState({
      error: error,
      errorInfo: errorInfo
    });
  }

  render() {
    if (this.state.hasError) {
      // You can render any custom fallback UI
      return (
        <div className="embedded-chatbot-error-boundary" style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '350px',
          padding: '16px',
          backgroundColor: '#fee',
          border: '1px solid #fcc',
          borderRadius: '8px',
          boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
          zIndex: 10000,
          fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Oxygen, Ubuntu, Cantarell, "Open Sans", "Helvetica Neue", sans-serif',
          fontSize: '14px',
          color: '#c33'
        }}>
          <h3 style={{ margin: '0 0 8px 0', color: '#a00' }}>Chatbot Error</h3>
          <p>We're sorry, but the chatbot encountered an error.</p>
          <button
            onClick={() => window.location.reload()}
            style={{
              background: '#0077cc',
              color: 'white',
              border: 'none',
              padding: '8px 16px',
              borderRadius: '4px',
              cursor: 'pointer',
              fontSize: '14px'
            }}
          >
            Reload Chatbot
          </button>
        </div>
      );
    }

    return this.props.children;
  }
}

export default ErrorBoundary;