import os
from typing import List, Dict, Any
from pathlib import Path
import logging

logger = logging.getLogger(__name__)


class ContentParser:
    """
    Utility class for parsing book content from various file formats
    """

    def __init__(self):
        self.supported_extensions = {'.txt', '.md', '.pdf', '.docx', '.html', '.htm'}

    def parse_file(self, file_path: str) -> List[Dict[str, Any]]:
        """
        Parse a single file and return a list of content chunks with metadata

        Args:
            file_path: Path to the file to parse

        Returns:
            List of dictionaries containing content and metadata
        """
        file_path = Path(file_path)
        extension = file_path.suffix.lower()

        if extension not in self.supported_extensions:
            logger.warning(f"Unsupported file extension: {extension} for file: {file_path}")
            return []

        try:
            if extension == '.txt':
                return self._parse_txt_file(file_path)
            elif extension == '.md':
                return self._parse_md_file(file_path)
            elif extension == '.pdf':
                return self._parse_pdf_file(file_path)
            elif extension in ['.docx']:
                return self._parse_docx_file(file_path)
            elif extension in ['.html', '.htm']:
                return self._parse_html_file(file_path)
            else:
                logger.warning(f"File extension {extension} not handled by parser: {file_path}")
                return []
        except Exception as e:
            logger.error(f"Error parsing file {file_path}: {e}")
            return []

    def parse_directory(self, directory_path: str) -> List[Dict[str, Any]]:
        """
        Parse all supported files in a directory and return content chunks

        Args:
            directory_path: Path to the directory to parse

        Returns:
            List of dictionaries containing content and metadata from all files
        """
        directory_path = Path(directory_path)
        all_chunks = []

        if not directory_path.exists():
            logger.error(f"Directory does not exist: {directory_path}")
            return []

        if not directory_path.is_dir():
            logger.error(f"Path is not a directory: {directory_path}")
            return []

        for file_path in directory_path.rglob('*'):
            if file_path.is_file() and file_path.suffix.lower() in self.supported_extensions:
                logger.info(f"Parsing file: {file_path}")
                file_chunks = self.parse_file(str(file_path))
                all_chunks.extend(file_chunks)

        return all_chunks

    def _parse_txt_file(self, file_path: Path) -> List[Dict[str, Any]]:
        """
        Parse a text file and return content chunks
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()

            # Split content into chunks (for now, just one chunk per file)
            # In a real implementation, you might want to chunk based on size
            return [{
                'content': content,
                'source_file': str(file_path),
                'file_type': 'txt',
                'metadata': {
                    'file_path': str(file_path),
                    'file_size': file_path.stat().st_size
                }
            }]
        except Exception as e:
            logger.error(f"Error reading text file {file_path}: {e}")
            return []

    def _parse_md_file(self, file_path: Path) -> List[Dict[str, Any]]:
        """
        Parse a markdown file and return content chunks
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()

            # For markdown, we could extract sections, but for now just return as one chunk
            return [{
                'content': content,
                'source_file': str(file_path),
                'file_type': 'md',
                'metadata': {
                    'file_path': str(file_path),
                    'file_size': file_path.stat().st_size
                }
            }]
        except Exception as e:
            logger.error(f"Error reading markdown file {file_path}: {e}")
            return []

    def _parse_pdf_file(self, file_path: Path) -> List[Dict[str, Any]]:
        """
        Parse a PDF file and return content chunks
        """
        try:
            # Import PDF parsing library
            import PyPDF2

            chunks = []
            with open(file_path, 'rb') as file:
                pdf_reader = PyPDF2.PdfReader(file)

                for page_num, page in enumerate(pdf_reader.pages):
                    text = page.extract_text()
                    if text.strip():  # Only add if text is not empty
                        chunks.append({
                            'content': text,
                            'source_file': str(file_path),
                            'page': page_num + 1,
                            'file_type': 'pdf',
                            'metadata': {
                                'file_path': str(file_path),
                                'file_size': file_path.stat().st_size,
                                'page_number': page_num + 1
                            }
                        })

            return chunks
        except ImportError:
            logger.error("PyPDF2 library not available for PDF parsing")
            logger.info("Install with: pip install PyPDF2")
            return []
        except Exception as e:
            logger.error(f"Error reading PDF file {file_path}: {e}")
            return []

    def _parse_docx_file(self, file_path: Path) -> List[Dict[str, Any]]:
        """
        Parse a DOCX file and return content chunks
        """
        try:
            # Import DOCX parsing library
            from docx import Document

            doc = Document(file_path)
            full_text = []

            for para in doc.paragraphs:
                full_text.append(para.text)

            content = '\n'.join(full_text)

            return [{
                'content': content,
                'source_file': str(file_path),
                'file_type': 'docx',
                'metadata': {
                    'file_path': str(file_path),
                    'file_size': file_path.stat().st_size
                }
            }]
        except ImportError:
            logger.error("python-docx library not available for DOCX parsing")
            logger.info("Install with: pip install python-docx")
            return []
        except Exception as e:
            logger.error(f"Error reading DOCX file {file_path}: {e}")
            return []

    def _parse_html_file(self, file_path: Path) -> List[Dict[str, Any]]:
        """
        Parse an HTML file and return content chunks
        """
        try:
            # Import HTML parsing library
            from bs4 import BeautifulSoup

            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()

            soup = BeautifulSoup(content, 'html.parser')

            # Extract text content, removing HTML tags
            text_content = soup.get_text(separator=' ')

            return [{
                'content': text_content,
                'source_file': str(file_path),
                'file_type': 'html',
                'metadata': {
                    'file_path': str(file_path),
                    'file_size': file_path.stat().st_size
                }
            }]
        except ImportError:
            logger.error("beautifulsoup4 library not available for HTML parsing")
            logger.info("Install with: pip install beautifulsoup4")
            return []
        except Exception as e:
            logger.error(f"Error reading HTML file {file_path}: {e}")
            return []

    def chunk_content(self, content: str, chunk_size: int = 512, overlap: int = 64) -> List[Dict[str, Any]]:
        """
        Chunk content into smaller pieces

        Args:
            content: The content to chunk
            chunk_size: Size of each chunk
            overlap: Overlap between chunks

        Returns:
            List of chunked content pieces
        """
        if not content:
            return []

        chunks = []
        start = 0

        while start < len(content):
            end = start + chunk_size
            chunk = content[start:end]

            chunks.append({
                'content': chunk,
                'start_pos': start,
                'end_pos': end
            })

            # Move start position by chunk_size - overlap to create overlap
            start = end - overlap if end < len(content) else end

            # If we've reached the end, break
            if start >= len(content):
                break

        return chunks


# Global instance for easy access
content_parser = ContentParser()