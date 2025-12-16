# Quickstart: Chatbot Display Fix

## Overview
This guide explains how to implement the fix for the chatbot display issue where API responses from the backend were not properly displayed in the frontend UI. The backend is working correctly, but the frontend wasn't rendering responses due to HTML formatting and XSS protection issues.

## Prerequisites
- Modern web browser
- Access to the frontend code
- Working backend API with RAG functionality

## Files to Modify

### 1. Frontend Script (`frontend/script.js`)
- Locate the `addMessage` function (around line 161)
- Update the message formatting logic to properly handle markdown and XSS protection
- Ensure proper handling of source information with 'score' field

## Implementation Steps

### Step 1: Update Message Formatting
```javascript
// In the addMessage function, replace the bot message handling:
if (sender === 'bot' && typeof content === 'object') {
    // Create paragraph element and set text content to avoid HTML injection issues
    const responsePara = document.createElement('p');
    // For better formatting, replace markdown-style **text** with <strong>text</strong>
    const formattedResponse = content.response.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');
    responsePara.innerHTML = formattedResponse;
    messageContent.appendChild(responsePara);

    if (content.sources && content.sources.length > 0) {
        const sourcesDiv = document.createElement('div');
        sourcesDiv.className = 'sources';
        sourcesDiv.innerHTML = `
            <h4>Sources:</h4>
            <ul class="sources-list">
                ${content.sources.map(source =>
                    `<li><strong>${source.chunk_id}:</strong> ${this.escapeHtml(source.content.substring(0, 100))}${source.content.length > 100 ? '...' : ''} (Score: ${(source.score * 100).toFixed(1)}%)</li>`
                ).join('')}
            </ul>
        `;
        messageContent.appendChild(sourcesDiv);
    }
}
```

### Step 2: Add HTML Escaping Helper
```javascript
// Add this helper function to prevent XSS:
escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}
```

### Step 3: Update Source Field Names
- Change references from 'confidence' to 'score' to match the actual API response format
- Ensure all source information displays correctly with the proper field names

## Testing

### Test Cases
1. **Basic response**: Verify simple responses display correctly
2. **Markdown formatting**: Test responses with `**bold**` formatting
3. **Source information**: Verify sources display with correct score values
4. **XSS protection**: Test that malicious HTML is properly escaped
5. **Long responses**: Ensure lengthy responses display without issues

### Expected Behavior
- Responses with markdown formatting render properly with bold text
- Source information displays with correct score percentages
- No XSS vulnerabilities are introduced
- Book content remains intact and properly displayed
- All responses from the backend API display correctly in the UI

## Deployment
1. Test the changes in a development environment
2. Verify that all response types display correctly
3. Ensure security measures are functioning properly
4. Deploy to production environment