import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import EmbeddedChatbot from './EmbeddedChatbot';

// Mock the CSS module
jest.mock('../styles/embedded-chatbot.css', () => ({}));

// Mock the fetch API
global.fetch = jest.fn(() =>
  Promise.resolve({
    ok: true,
    json: () =>
      Promise.resolve({
        response: 'Test response',
        sources: [],
        status: 'success',
        timestamp: new Date().toISOString(),
      }),
  })
);

describe('EmbeddedChatbot', () => {
  const mockPageContext = {
    pageId: '/test-page',
    pageTitle: 'Test Page',
    pageContentPreview: 'Test content preview',
    sectionInfo: {
      currentHeading: 'Test Heading',
      url: 'http://localhost/test-page',
      pathname: '/test-page',
    },
    lastUpdated: new Date().toISOString(),
  };

  beforeEach(() => {
    fetch.mockClear();
  });

  test('renders floating button by default (closed state)', () => {
    render(<EmbeddedChatbot pageContext={mockPageContext} />);

    // Check that the floating button is present
    const floatingButton = screen.getByLabelText('Open chatbot assistant');
    expect(floatingButton).toBeInTheDocument();
    expect(floatingButton).toHaveClass('chatbot-floating-button');

    // Check that the chat widget is not visible initially
    const chatWidget = screen.queryByLabelText('Chat with Humanoid Robotics Assistant');
    expect(chatWidget).not.toBeInTheDocument();
  });

  test('toggles visibility when floating button is clicked', async () => {
    render(<EmbeddedChatbot pageContext={mockPageContext} />);

    // Initially, the floating button should be visible
    const floatingButton = screen.getByLabelText('Open chatbot assistant');
    expect(floatingButton).toBeInTheDocument();

    // Click the floating button to open the widget
    fireEvent.click(floatingButton);

    // Wait for the widget to appear
    await waitFor(() => {
      const chatWidget = screen.getByLabelText('Chat with Humanoid Robotics Assistant');
      expect(chatWidget).toBeInTheDocument();
    });

    // Click the close button to close the widget
    const closeBtn = screen.getByLabelText('Close chat');
    fireEvent.click(closeBtn);

    // Wait for the widget to disappear and floating button to reappear
    await waitFor(() => {
      expect(screen.getByLabelText('Open chatbot assistant')).toBeInTheDocument();
    });
  });

  test('closes when clicking outside the widget', async () => {
    render(<EmbeddedChatbot pageContext={mockPageContext} />);

    // Open the chat widget
    const floatingButton = screen.getByLabelText('Open chatbot assistant');
    fireEvent.click(floatingButton);

    await waitFor(() => {
      expect(screen.getByLabelText('Chat with Humanoid Robotics Assistant')).toBeInTheDocument();
    });

    // Click outside the widget (on the document body)
    fireEvent.mouseDown(document.body);

    // Wait for the widget to close
    await waitFor(() => {
      expect(screen.getByLabelText('Open chatbot assistant')).toBeInTheDocument();
    });
  });

  test('initial state is closed', () => {
    render(<EmbeddedChatbot pageContext={mockPageContext} />);

    // Verify that the floating button is shown (not the chat widget)
    expect(screen.getByLabelText('Open chatbot assistant')).toBeInTheDocument();

    // The chat widget should not be in the document initially
    expect(screen.queryByLabelText('Chat with Humanoid Robotics Assistant')).not.toBeInTheDocument();
  });

  test('handles keyboard shortcuts', async () => {
    render(<EmbeddedChatbot pageContext={mockPageContext} />);

    // Initially closed - floating button should be visible
    const floatingButton = screen.getByLabelText('Open chatbot assistant');
    expect(floatingButton).toBeInTheDocument();

    // Simulate pressing Ctrl/Cmd + Shift + C to open
    fireEvent.keyDown(document, {
      key: 'C',
      ctrlKey: true,
      shiftKey: true,
    });

    await waitFor(() => {
      expect(screen.getByLabelText('Chat with Humanoid Robotics Assistant')).toBeInTheDocument();
    });

    // Now test Escape key to close
    fireEvent.keyDown(document, { key: 'Escape' });

    await waitFor(() => {
      expect(screen.getByLabelText('Open chatbot assistant')).toBeInTheDocument();
    });
  });

  test('maintains existing chat functionality when open', async () => {
    render(<EmbeddedChatbot pageContext={mockPageContext} />);

    // Open the chat widget
    fireEvent.click(screen.getByLabelText('Open chatbot assistant'));

    await waitFor(() => {
      expect(screen.getByLabelText('Chat with Humanoid Robotics Assistant')).toBeInTheDocument();
    });

    // Find the input field and submit button
    const input = screen.getByLabelText('Type your question');
    const submitButton = screen.getByText('Send');

    // Type a message
    fireEvent.change(input, { target: { value: 'Hello, world!' } });

    // Submit the message
    fireEvent.click(submitButton);

    // Wait for the API call to complete
    await waitFor(() => {
      expect(fetch).toHaveBeenCalledTimes(1);
    });

    // Check that the message was sent correctly
    expect(fetch).toHaveBeenCalledWith('http://localhost:8000/api/ask', expect.any(Object));
  });

  test('shows loading state during API calls', async () => {
    // Mock a delayed response
    fetch.mockImplementationOnce(() =>
      new Promise(resolve =>
        setTimeout(() => resolve({
          ok: true,
          json: () => Promise.resolve({
            response: 'Delayed response',
            sources: [],
            status: 'success',
            timestamp: new Date().toISOString(),
          }),
        }), 100)
      )
    );

    render(<EmbeddedChatbot pageContext={mockPageContext} />);

    // Open the chat widget
    fireEvent.click(screen.getByLabelText('Open chatbot assistant'));

    await waitFor(() => {
      expect(screen.getByLabelText('Chat with Humanoid Robotics Assistant')).toBeInTheDocument();
    });

    // Type and submit a message
    const input = screen.getByLabelText('Type your question');
    fireEvent.change(input, { target: { value: 'Test message' } });
    fireEvent.click(screen.getByText('Send'));

    // Check that loading indicator appears
    expect(screen.getByText('Thinking...')).toBeInTheDocument();

    // Wait for response
    await waitFor(() => {
      expect(screen.getByText('Delayed response')).toBeInTheDocument();
    });
  });
});