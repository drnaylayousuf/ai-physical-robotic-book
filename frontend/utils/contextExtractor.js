/**
 * Utility functions for extracting page context
 */

export const extractPageContext = () => {
  return {
    pageId: window.location.pathname,
    pageTitle: document.title,
    pageContentPreview: extractContentPreview(),
    sectionInfo: extractSectionInfo(),
    lastUpdated: new Date().toISOString(),
  };
};

export const extractContentPreview = (maxLength = 500) => {
  // Extract text content from the main content area
  let content = '';

  // Try to find the main content area
  const mainContent = document.querySelector('main, .main, .container, .content, article');
  if (mainContent) {
    content = mainContent.textContent || mainContent.innerText;
  } else {
    // Fallback to body content
    content = document.body.textContent || document.body.innerText;
  }

  // Clean up the content
  content = content.replace(/\s+/g, ' ').trim();

  // Limit length and return
  return content.substring(0, maxLength);
};

export const extractSectionInfo = () => {
  // Try to find current section information
  const heading = document.querySelector('h1, h2, h3, h4, h5, h6');
  const sectionInfo = {
    currentHeading: heading ? heading.textContent.trim() : '',
    url: window.location.href,
    pathname: window.location.pathname,
  };

  return sectionInfo;
};

export const extractMetadata = () => {
  // Extract metadata from the page
  const metaDescription = document.querySelector('meta[name="description"]');
  const metaKeywords = document.querySelector('meta[name="keywords"]');

  return {
    description: metaDescription ? metaDescription.getAttribute('content') : '',
    keywords: metaKeywords ? metaKeywords.getAttribute('content') : '',
    author: document.querySelector('meta[name="author"]')?.getAttribute('content') || '',
    title: document.title,
  };
};

export const getPageStructure = () => {
  // Extract basic page structure information
  const headings = Array.from(document.querySelectorAll('h1, h2, h3, h4, h5, h6')).map(h => ({
    level: h.tagName.charAt(1),
    text: h.textContent.trim(),
    id: h.id || null,
  }));

  return {
    headings,
    totalHeadings: headings.length,
    hasNavigation: !!document.querySelector('nav, .nav, .navigation'),
    hasSidebar: !!document.querySelector('aside, .sidebar'),
  };
};