import React, { useState, useEffect } from 'react';
import Layout from '@theme-original/Layout';
import EmbeddedChatbot from '@site/frontend/components/EmbeddedChatbot';
import ErrorBoundary from '@site/frontend/components/ErrorBoundary';
import { extractPageContext } from '@site/frontend/utils/contextExtractor';
import AuthProvider from '../contexts/AuthProvider';

// Wrapper component that adds the embedded chatbot and auth provider to all pages
export default function LayoutWrapper(props) {
  // Extract page context when the component mounts
  const [pageContext, setPageContext] = useState(null);

  useEffect(() => {
    try {
      // Extract context from the current page
      const context = extractPageContext();
      setPageContext(context);
    } catch (error) {
      console.warn('Could not extract page context:', error);
      setPageContext({
        pageId: window.location.pathname,
        pageTitle: document.title,
        pageContentPreview: '',
        sectionInfo: { currentHeading: '', url: window.location.href, pathname: window.location.pathname },
        lastUpdated: new Date().toISOString(),
      });
    }
  }, []);

  return (
    <AuthProvider>
      <Layout {...props}>
        {/* Render the main content */}
        {props.children}

        {/* Add the embedded chatbot with error boundary */}
        <ErrorBoundary>
          <EmbeddedChatbot pageContext={pageContext} />
        </ErrorBoundary>

      </Layout>
    </AuthProvider>
  );
}