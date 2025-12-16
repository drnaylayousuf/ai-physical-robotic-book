/**
 * Utility functions for text selection functionality
 */

export const getSelectedText = () => {
  const selection = window.getSelection();
  return selection.toString().trim();
};

export const getSelectedRange = () => {
  const selection = window.getSelection();
  if (selection.rangeCount > 0) {
    return selection.getRangeAt(0);
  }
  return null;
};

export const highlightSelectedText = (color = '#ffff99') => {
  const selection = window.getSelection();
  if (selection.rangeCount > 0) {
    const range = selection.getRangeAt(0);
    const span = document.createElement('span');
    span.style.backgroundColor = color;
    span.className = 'chatbot-text-selection-highlight';

    range.surroundContents(span);
    return span;
  }
  return null;
};

export const clearHighlights = () => {
  const highlights = document.querySelectorAll('.chatbot-text-selection-highlight');
  highlights.forEach(highlight => {
    const parent = highlight.parentNode;
    while (highlight.firstChild) {
      parent.insertBefore(highlight.firstChild, highlight);
    }
    parent.removeChild(highlight);
  });
};

export const addTextSelectionListener = (callback) => {
  const handleSelection = () => {
    const selectedText = getSelectedText();
    if (selectedText) {
      callback(selectedText);
    }
  };

  // Listen for mouse up and keyboard events to detect text selection
  document.addEventListener('mouseup', handleSelection);
  document.addEventListener('keyup', (e) => {
    if (e.key === 'Escape') {
      window.getSelection().removeAllRanges();
    }
  });

  // Return cleanup function
  return () => {
    document.removeEventListener('mouseup', handleSelection);
  };
};

export const createTextSelectionTooltip = (selectedText, position) => {
  // Create a tooltip element to show selected text options
  const tooltip = document.createElement('div');
  tooltip.className = 'chatbot-selection-tooltip';
  tooltip.style.position = 'absolute';
  tooltip.style.left = `${position.x}px`;
  tooltip.style.top = `${position.y}px`;
  tooltip.style.backgroundColor = '#333';
  tooltip.style.color = 'white';
  tooltip.style.padding = '8px 12px';
  tooltip.style.borderRadius = '4px';
  tooltip.style.zIndex = '10000';
  tooltip.style.fontSize = '14px';
  tooltip.innerHTML = `
    <button class="ask-about-text-btn" style="background: none; border: none; color: white; cursor: pointer; text-decoration: underline;">
      Ask about this text
    </button>
  `;

  // Add click event to the button
  const askButton = tooltip.querySelector('.ask-about-text-btn');
  askButton.onclick = () => {
    if (typeof callback === 'function') {
      callback(selectedText);
    }
    document.body.removeChild(tooltip);
  };

  document.body.appendChild(tooltip);

  // Remove tooltip after a delay
  setTimeout(() => {
    if (tooltip.parentNode) {
      document.body.removeChild(tooltip);
    }
  }, 3000);

  return tooltip;
};