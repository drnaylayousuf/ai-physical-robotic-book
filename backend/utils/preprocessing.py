import re
from typing import List, Dict, Optional
import logging
from pathlib import Path

logger = logging.getLogger(__name__)

class TextPreprocessor:
    """
    Utility class for preprocessing text content from various sources
    """

    @staticmethod
    def clean_text(text: str) -> str:
        """
        Clean and normalize text content
        """
        # Remove extra whitespace
        text = ' '.join(text.split())

        # Remove special characters while preserving important punctuation
        # Keep letters, numbers, basic punctuation, and whitespace
        text = re.sub(r'[^\w\s\.\,\!\?\;\:\-\(\)\[\]\{\}\'\"]', ' ', text)

        # Remove extra spaces created by the substitution
        text = ' '.join(text.split())

        return text.strip()

    @staticmethod
    def extract_text_from_markdown(content: str) -> str:
        """
        Extract plain text from markdown content
        """
        # Remove markdown headers
        content = re.sub(r'^#+\s+', '', content, flags=re.MULTILINE)

        # Remove markdown links and images
        content = re.sub(r'!\[.*?\]\(.*?\)', '', content)
        content = re.sub(r'\[.*?\]\(.*?\)', '', content)

        # Remove markdown bold and italic markers
        content = re.sub(r'\*\*(.*?)\*\*', r'\1', content)
        content = re.sub(r'\*(.*?)\*', r'\1', content)
        content = re.sub(r'__(.*?)__', r'\1', content)
        content = re.sub(r'_(.*?)_', r'\1', content)

        # Remove code blocks
        content = re.sub(r'```.*?```', '', content, flags=re.DOTALL)
        content = re.sub(r'`.*?`', '', content)

        # Remove blockquotes
        content = re.sub(r'^\s*>\s*', '', content, flags=re.MULTILINE)

        # Remove horizontal rules
        content = re.sub(r'^\s*[-*_]{3,}\s*$', '', content, flags=re.MULTILINE)

        # Clean up the text
        content = TextPreprocessor.clean_text(content)

        return content

    @staticmethod
    def extract_text_from_html(content: str) -> str:
        """
        Extract plain text from HTML content
        """
        # Remove HTML tags
        content = re.sub(r'<[^>]+>', ' ', content)

        # Unescape HTML entities
        content = content.replace('&nbsp;', ' ')
        content = content.replace('&amp;', '&')
        content = content.replace('&lt;', '<')
        content = content.replace('&gt;', '>')
        content = content.replace('&quot;', '"')
        content = content.replace('&#x27;', "'")

        # Clean up the text
        content = TextPreprocessor.clean_text(content)

        return content

    @staticmethod
    def split_into_paragraphs(text: str) -> List[str]:
        """
        Split text into paragraphs based on double line breaks
        """
        paragraphs = []
        for para in text.split('\n\n'):
            para = para.strip()
            if para:
                paragraphs.append(para)
        return paragraphs

    @staticmethod
    def count_paragraphs(text: str) -> int:
        """
        Count the number of paragraphs in text
        """
        paragraphs = TextPreprocessor.split_into_paragraphs(text)
        return len(paragraphs)

    @staticmethod
    def extract_chapter_metadata(content: str, file_path: str) -> Dict:
        """
        Extract chapter metadata from content
        """
        # Extract chapter title (try different patterns)
        import re

        # Look for markdown headers (h1 or h2)
        title_match = re.search(r'^(?:#|\n#|#\s)([^\n]+)', content, re.MULTILINE)
        if not title_match:
            title_match = re.search(r'^(?:##|\n##|##\s)([^\n]+)', content, re.MULTILINE)

        if title_match:
            title = title_match.group(1).strip()
        else:
            # Use filename as title if no header found
            title = Path(file_path).stem.replace('_', ' ').replace('-', ' ').title()

        # Count paragraphs
        paragraph_count = TextPreprocessor.count_paragraphs(content)

        # Create a simple summary (first 200 characters)
        content_summary = content.strip()[:200]
        if len(content) > 200:
            content_summary += "..."

        return {
            "chapter_title": title,
            "file_path": file_path,
            "content_summary": content_summary,
            "paragraph_count": paragraph_count
        }

    @staticmethod
    def read_file_content(file_path: str) -> str:
        """
        Read content from various file formats
        """
        path = Path(file_path)

        if not path.exists():
            raise FileNotFoundError(f"File not found: {file_path}")

        # Determine file type based on extension
        file_extension = path.suffix.lower()

        with open(path, 'r', encoding='utf-8') as file:
            content = file.read()

        # Preprocess content based on file type
        if file_extension == '.md':
            content = TextPreprocessor.extract_text_from_markdown(content)
        elif file_extension == '.html' or file_extension == '.htm':
            content = TextPreprocessor.extract_text_from_html(content)
        else:
            # For other file types, just clean the text
            content = TextPreprocessor.clean_text(content)

        return content

    @staticmethod
    def read_directory_content(directory_path: str, extensions: Optional[List[str]] = None) -> Dict[str, str]:
        """
        Read content from all files in a directory
        """
        if extensions is None:
            extensions = ['.txt', '.md', '.html', '.htm']

        directory = Path(directory_path)
        content_map = {}

        if not directory.exists():
            raise FileNotFoundError(f"Directory not found: {directory_path}")

        for file_path in directory.rglob('*'):
            if file_path.is_file() and file_path.suffix.lower() in extensions:
                try:
                    content = TextPreprocessor.read_file_content(str(file_path))
                    relative_path = str(file_path.relative_to(directory))
                    content_map[relative_path] = content
                except Exception as e:
                    logger.warning(f"Failed to read file {file_path}: {e}")

        return content_map

    @staticmethod
    def chunk_text(text: str, chunk_size: int = 512, overlap: int = 64) -> List[str]:
        """
        Split text into overlapping chunks
        """
        chunks = []
        start = 0

        while start < len(text):
            end = start + chunk_size
            chunk = text[start:end]
            chunks.append(chunk)

            # Move start position by chunk_size - overlap to create overlap
            start = end - overlap if end < len(text) else end

            # If we've reached the end, break
            if start >= len(text):
                break

        return chunks