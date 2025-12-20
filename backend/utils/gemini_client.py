import os
import logging
from typing import Optional, List, Dict, Any
from dotenv import load_dotenv
import google.generativeai as genai

# Load environment variables
load_dotenv()

# Set up logging
logger = logging.getLogger(__name__)

class GeminiClient:
    """
    Client class for interacting with Google's Generative AI (Gemini)
    """

    def __init__(self, api_key: Optional[str] = None, model_name: str = "gemini-2.5-flash"):
        """
        Initialize the Gemini client

        Args:
            api_key: Google API key for Gemini (defaults to GEMINI_API_KEY environment variable)
            model_name: Name of the Gemini model to use (defaults to gemini-2.5-flash)
        """
        self.api_key = api_key or os.getenv("GEMINI_API_KEY")
        self.model_name = model_name or os.getenv("GEMINI_MODEL", "gemini-2.5-flash")

        if not self.api_key:
            raise ValueError("GEMINI_API_KEY environment variable is required")

        # Configure the API key
        genai.configure(api_key=self.api_key)

        # Initialize the model
        self.model = genai.GenerativeModel(self.model_name)

        logger.info(f"Initialized Gemini client with model: {self.model_name}")

    def generate_text(self, prompt: str, context: Optional[str] = None, max_output_tokens: int = 1000) -> Optional[str]:
        """
        Generate text based on a prompt and optional context

        Args:
            prompt: The main prompt/question to send to Gemini
            context: Additional context to provide to the model
            max_output_tokens: Maximum number of tokens in the output

        Returns:
            Generated text response or None if an error occurs
        """
        try:
            # Combine context and prompt if context is provided
            if context:
                full_prompt = f"Context: {context}\n\nQuestion: {prompt}"
            else:
                full_prompt = prompt

            # Generate content
            response = self.model.generate_content(
                full_prompt,
                generation_config=genai.types.GenerationConfig(
                    max_output_tokens=max_output_tokens,
                    temperature=0.7,  # Balanced creativity and consistency
                )
            )

            # Extract and return the text
            if response and response.text:
                return response.text.strip()
            else:
                logger.warning("Gemini returned empty response")
                return None

        except Exception as e:
            logger.error(f"Error generating text with Gemini: {str(e)}")
            return None

    def generate_with_safety_settings(self, prompt: str, context: Optional[str] = None) -> Optional[str]:
        """
        Generate text with specific safety settings to handle various content types

        Args:
            prompt: The main prompt/question to send to Gemini
            context: Additional context to provide to the model

        Returns:
            Generated text response or None if an error occurs
        """
        try:
            # Combine context and prompt if context is provided
            if context:
                full_prompt = f"Context: {context}\n\nQuestion: {prompt}"
            else:
                full_prompt = prompt

            # Define safety settings
            safety_settings = [
                {"category": "HARM_CATEGORY_HARASSMENT", "threshold": "BLOCK_ONLY_HIGH"},
                {"category": "HARM_CATEGORY_HATE_SPEECH", "threshold": "BLOCK_ONLY_HIGH"},
                {"category": "HARM_CATEGORY_SEXUALLY_EXPLICIT", "threshold": "BLOCK_ONLY_HIGH"},
                {"category": "HARM_CATEGORY_DANGEROUS_CONTENT", "threshold": "BLOCK_ONLY_HIGH"},
            ]

            # Generate content with safety settings
            response = self.model.generate_content(
                full_prompt,
                safety_settings=safety_settings,
                generation_config=genai.types.GenerationConfig(
                    temperature=0.7,
                    max_output_tokens=1000
                )
            )

            # Extract and return the text
            if response and response.text:
                return response.text.strip()
            else:
                logger.warning("Gemini returned empty response with safety settings")
                return None

        except Exception as e:
            logger.error(f"Error generating text with safety settings: {str(e)}")
            return None

    def embed_content(self, content: str) -> Optional[List[float]]:
        """
        Create embeddings for the given content using Gemini's embedding capabilities

        Args:
            content: Text content to create embeddings for

        Returns:
            List of embedding values or None if an error occurs
        """
        try:
            # Use the embedding model
            embedding_model = "embedding-001"  # Use the appropriate embedding model
            result = genai.embed_content(
                model=embedding_model,
                content=content,
                task_type="retrieval_document"  # or "retrieval_query" for search queries
            )

            return result['embedding'] if 'embedding' in result else None

        except Exception as e:
            logger.error(f"Error creating embeddings with Gemini: {str(e)}")
            return None

    def chat_session(self, system_instruction: Optional[str] = None):
        """
        Create a chat session for multi-turn conversations

        Args:
            system_instruction: Optional system instructions to guide the model's behavior

        Returns:
            A chat session object for multi-turn conversations
        """
        try:
            # Start a new chat session
            chat = self.model.start_chat()

            if system_instruction:
                # Send system instruction as the first message
                chat.send_message(system_instruction)

            return chat

        except Exception as e:
            logger.error(f"Error creating chat session: {str(e)}")
            return None

# Global instance for easy access
try:
    gemini_client = GeminiClient()
except ValueError as e:
    # If API key is not configured, set to None to handle gracefully elsewhere
    logger.warning(f"Gemini client not initialized: {str(e)}")
    gemini_client = None