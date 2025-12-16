/**
 * Text Selection and Highlighting Tool
 * Provides functionality for selecting text in the book content and passing it to the backend
 */
class TextSelector {
    constructor(containerId, options = {}) {
        this.container = document.getElementById(containerId);
        this.options = {
            highlightColor: options.highlightColor || '#ffff00',
            highlightClass: options.highlightClass || 'selected-text',
            ...options
        };
        this.selectedText = '';
        this.selectionRange = null;

        this.init();
    }

    init() {
        if (!this.container) {
            console.error('Text selector container not found');
            return;
        }

        // Add event listeners for text selection
        this.container.addEventListener('mouseup', (e) => {
            this.handleTextSelection(e);
        });

        // Add CSS for highlighting if not already present
        this.addHighlightStyles();
    }

    handleTextSelection(event) {
        const selection = window.getSelection();

        if (selection.toString().trim() === '') {
            // No text selected
            this.selectedText = '';
            this.selectionRange = null;
            return;
        }

        const selectedText = selection.toString().trim();

        // Validate selection length
        if (selectedText.length < 10) {
            alert('Please select more text (minimum 10 characters).');
            return;
        }

        if (selectedText.length > 1000) {
            alert('Selected text is too long. Please select a shorter portion (maximum 1000 characters).');
            return;
        }

        // Validate that the selection is within the container element
        const range = selection.getRangeAt(0);
        if (!this.isSelectionWithinContainer(range)) {
            console.log('Selection is not within the book content container, ignoring.');
            return;
        }

        // Store the selection range for potential highlighting
        this.selectionRange = range.cloneRange();
        this.selectedText = selectedText;

        // Trigger custom event for other components
        this.dispatchSelectionEvent(selectedText);
    }

    // Check if the selection range is within the container element
    isSelectionWithinContainer(range) {
        if (!range) return false;

        // Get the start and end containers of the selection
        const startContainer = range.startContainer;
        const endContainer = range.endContainer;

        // Check if both start and end containers are within our target container
        const isStartWithin = this.container.contains(startContainer) || startContainer === this.container;
        const isEndWithin = this.container.contains(endContainer) || endContainer === this.container;

        return isStartWithin && isEndWithin;
    }

    dispatchSelectionEvent(selectedText) {
        const event = new CustomEvent('textSelected', {
            detail: {
                text: selectedText,
                range: this.selectionRange
            }
        });
        document.dispatchEvent(event);
    }

    highlightSelectedText() {
        if (!this.selectionRange) return;

        // Remove existing highlights
        this.removeHighlights();

        // Create a new element to wrap the selected text
        const highlightSpan = document.createElement('span');
        highlightSpan.className = this.options.highlightClass;
        highlightSpan.style.backgroundColor = this.options.highlightColor;
        highlightSpan.style.padding = '2px';

        try {
            // Surround the content with the highlight span
            this.selectionRange.surroundContents(highlightSpan);
        } catch (e) {
            // If surrounding fails (e.g., partially selected elements),
            // we'll handle it by extracting the content and reinserting
            console.warn('Could not directly surround content, using alternative method:', e);

            // Alternative method: extract content and reinsert
            const content = this.selectionRange.extractContents();
            highlightSpan.appendChild(content);
            this.selectionRange.insertNode(highlightSpan);
        }
    }

    removeHighlights() {
        const highlights = this.container.querySelectorAll(`.${this.options.highlightClass}`);
        highlights.forEach(highlight => {
            // Move the children of the highlight element out of it
            const parent = highlight.parentNode;
            while (highlight.firstChild) {
                parent.insertBefore(highlight.firstChild, highlight);
            }
            // Remove the empty highlight element
            parent.removeChild(highlight);
        });
    }

    getSelectedText() {
        return this.selectedText;
    }

    addHighlightStyles() {
        // Check if styles are already added
        if (document.getElementById('text-selector-styles')) {
            return;
        }

        const style = document.createElement('style');
        style.id = 'text-selector-styles';
        style.textContent = `
            .${this.options.highlightClass} {
                background-color: ${this.options.highlightColor};
                padding: 2px;
                border-radius: 2px;
                transition: background-color 0.3s ease;
            }

            .${this.options.highlightClass}:hover {
                background-color: #ffff66;
            }
        `;
        document.head.appendChild(style);
    }

    // Method to clear the current selection
    clearSelection() {
        this.selectedText = '';
        this.selectionRange = null;
        this.removeHighlights();

        // Clear browser selection
        if (window.getSelection) {
            window.getSelection().removeAllRanges();
        }
    }

    // Method to get word count of selected text
    getWordCount() {
        if (!this.selectedText) return 0;
        return this.selectedText.trim().split(/\s+/).filter(word => word.length > 0).length;
    }

    // Method to get character count of selected text
    getCharacterCount() {
        return this.selectedText ? this.selectedText.length : 0;
    }
}

// Export for use in other modules (if using modules)
if (typeof module !== 'undefined' && module.exports) {
    module.exports = TextSelector;
}

// Initialize text selector on elements with specific class
document.addEventListener('DOMContentLoaded', () => {
    const selectableElements = document.querySelectorAll('.selectable-text, .book-content, [data-text-selectable]');

    selectableElements.forEach(element => {
        const selector = new TextSelector(element.id, {
            highlightColor: '#ffff0080' // Semi-transparent yellow
        });
    });
});