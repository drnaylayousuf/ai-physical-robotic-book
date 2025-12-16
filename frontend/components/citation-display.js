/**
 * Citation Display Component
 * Displays source citations with chunk IDs and paragraph references
 */
class CitationDisplay {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.citations = [];
    }

    render(citations) {
        if (!this.container) {
            console.error('Citation display container not found');
            return;
        }

        this.citations = citations || [];

        // Clear existing content
        this.container.innerHTML = '';

        if (this.citations.length === 0) {
            this.container.style.display = 'none';
            return;
        }

        this.container.style.display = 'block';

        // Create citation container
        const citationDiv = document.createElement('div');
        citationDiv.className = 'citation-container';

        // Add title
        const title = document.createElement('h4');
        title.textContent = 'Sources & Citations';
        title.className = 'citation-title';
        citationDiv.appendChild(title);

        // Create citations list
        const citationsList = document.createElement('ul');
        citationsList.className = 'citations-list';

        this.citations.forEach((citation, index) => {
            const listItem = document.createElement('li');
            listItem.className = 'citation-item';

            // Format confidence as percentage
            const confidencePercent = (citation.confidence * 100).toFixed(1);

            // Create citation content
            listItem.innerHTML = `
                <div class="citation-header">
                    <span class="citation-id">${citation.chunk_id}</span>
                    <span class="citation-confidence">Confidence: ${confidencePercent}%</span>
                </div>
                <div class="citation-content">
                    <p>${citation.content.substring(0, 200)}${citation.content.length > 200 ? '...' : ''}</p>
                </div>
            `;

            citationsList.appendChild(listItem);
        });

        citationDiv.appendChild(citationsList);
        this.container.appendChild(citationDiv);
    }

    addCitation(citation) {
        this.citations.push(citation);
        this.render(this.citations);
    }

    clearCitations() {
        this.citations = [];
        this.render([]);
    }

    // Static method to create a formatted citation element
    static createCitationElement(citation) {
        const citationElement = document.createElement('div');
        citationElement.className = 'citation';

        const confidencePercent = (citation.confidence * 100).toFixed(1);

        citationElement.innerHTML = `
            <div class="citation-header">
                <span class="citation-id">${citation.chunk_id}</span>
                <span class="citation-confidence">Confidence: ${confidencePercent}%</span>
            </div>
            <div class="citation-content">
                <p>${citation.content}</p>
            </div>
        `;

        return citationElement;
    }
}

// Export for use in other modules (if using modules)
if (typeof module !== 'undefined' && module.exports) {
    module.exports = CitationDisplay;
}