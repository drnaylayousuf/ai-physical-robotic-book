from typing import List, Dict, Optional, Tuple, Any
from pydantic import BaseModel
import logging
from fastapi import HTTPException
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct, Filter, FieldCondition, MatchValue
from ..config.settings import settings
from ..utils.embeddings import EmbeddingService
from ..utils.monitoring import monitoring

logger = logging.getLogger(__name__)

class ChatRequest(BaseModel):
    question: str
    mode: str = "full_book"
    selected_text: Optional[str] = None
    page_context: Optional[Dict] = None
    user_id: Optional[str] = None

class ChatResponse(BaseModel):
    question: str
    answer: str
    sources: List[Dict[str, Any]]
    mode: str
    timestamp: str
    processing_time_ms: int

class RAGResponse(BaseModel):
    response: str
    sources: List[Dict]
    references: List[str]

class RAGModel:
    """
    RAG (Retrieval-Augmented Generation) model implementation
    Implements the clean → chunk → embed → store → retrieve → generate flow
    """

    def __init__(self):
        # Initialize embedding service with error handling
        try:
            self.embedding_service = EmbeddingService()
            logger.info("Embedding service initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize embedding service: {e}")
            raise ValueError(f"Embedding service initialization failed: {e}")

        # Initialize Qdrant client for HTTP connection - Qdrant Cloud only (no fallback)
        if not settings.QDRANT_URL or not settings.QDRANT_API_KEY:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required for Qdrant Cloud connection")

        try:
            # Initialize Qdrant client for Qdrant Cloud with authentication
            self.qdrant_client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                timeout=10  # Reduced timeout to prevent long blocking during initialization
            )

            # Test the connection with a timeout to avoid blocking during initialization
            import time
            start_time = time.time()
            self.qdrant_client.get_collections()
            connection_time = time.time() - start_time
            logger.info(f"Successfully connected to Qdrant Cloud at {settings.QDRANT_URL} (connection time: {connection_time:.2f}s)")

            # Verify collection exists before proceeding
            collection_name = settings.QDRANT_COLLECTION_NAME
            try:
                collection_info = self.qdrant_client.get_collection(collection_name)
                logger.info(f"Verified collection '{collection_name}' exists with {collection_info.points_count} points")
            except Exception as e:
                logger.info(f"Collection '{collection_name}' does not exist yet: {e}. It will be created when data is ingested.")
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant Cloud at {settings.QDRANT_URL}: {e}")
            # No fallback - this is a critical error that should stop the application
            raise ConnectionError(f"Could not connect to Qdrant Cloud: {e}")

        # Initialize Gemini generative model with multiple model support
        self.generative_model = None
        self.gemini_models = []
        self.current_model_index = 0

        if settings.GEMINI_API_KEY:
            try:
                import google.generativeai as genai
                genai.configure(api_key=settings.GEMINI_API_KEY)

                # Parse the models list from settings
                model_names = [model.strip() for model in settings.GEMINI_MODELS.split(',')]

                # Create model instances for each available model
                for model_name in model_names:
                    if model_name.strip():  # Only add non-empty model names
                        model_instance = genai.GenerativeModel(model_name.strip())
                        self.gemini_models.append(model_instance)

                # Use the first model as the primary
                if self.gemini_models:
                    self.generative_model = self.gemini_models[0]
                    logger.info(f"Gemini generative models configured: {model_names}")
                else:
                    # Fallback to the default model if GEMINI_MODELS is empty
                    self.generative_model = genai.GenerativeModel(settings.GEMINI_MODEL)
                    logger.info(f"Gemini generative model {settings.GEMINI_MODEL} configured (fallback)")
            except Exception as e:
                logger.error(f"Failed to configure Gemini generative models: {e}")
                # Instead of failing silently, log a more descriptive error
                logger.warning("Gemini API is not configured properly. The system will attempt to function but may have limited capabilities.")
        else:
            logger.warning("GEMINI_API_KEY not set. Gemini model will not be available. The system will attempt to function but may have limited capabilities.")

    async def clean_text(self, text: str) -> str:
        """
        Clean and normalize text content
        """
        # Remove extra whitespace
        text = ' '.join(text.split())
        # Remove special characters if needed
        # Add more cleaning steps as needed
        return text.strip()

    async def chunk_text(self, text: str, chunk_size: int = None, overlap: int = None) -> List[str]:
        """
        Split text into chunks with specified size and overlap
        """
        if not text or not text.strip():
            return []  # Return empty list if text is empty or only whitespace

        if chunk_size is None:
            chunk_size = settings.CHUNK_SIZE
        if overlap is None:
            overlap = settings.CHUNK_OVERLAP

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

    async def embed_chunks(self, chunks: List[str]) -> List[List[float]]:
        """
        Generate embeddings for text chunks
        """
        embeddings = []
        for chunk in chunks:
            try:
                embedding = await self.embedding_service.generate_embedding(chunk)
                embeddings.append(embedding)
            except Exception as e:
                logger.error(f"Failed to generate embedding for chunk: {e}")
                # Use a default embedding as fallback
                embeddings.append([0.0] * settings.EMBEDDING_DIMENSION)
        return embeddings

    async def store_chunks(self, chunks: List[str], embeddings: List[List[float]], collection_name: str = None) -> List[str]:
        """
        Store chunks and embeddings in vector database (Qdrant Cloud only - no fallback)
        """
        if collection_name is None:
            collection_name = settings.QDRANT_COLLECTION_NAME

        # Use Qdrant client - no fallback option
        try:
            collection_info = self.qdrant_client.get_collection(collection_name)
            # Verify the vector size matches
            expected_size = len(embeddings[0]) if embeddings else self.embedding_service.get_embedding_size()
            # Access the vector size properly - it might be in different structures depending on Qdrant version
            if hasattr(collection_info.config.params, 'vectors'):
                # For newer Qdrant versions with named vectors
                if isinstance(collection_info.config.params.vectors, dict):
                    # If vectors is a dict, get the first vector configuration
                    first_vector_name = next(iter(collection_info.config.params.vectors))
                    actual_size = collection_info.config.params.vectors[first_vector_name].size
                else:
                    # If vectors is a VectorParams object
                    actual_size = collection_info.config.params.vectors.size
            elif hasattr(collection_info.config.params, 'size'):
                # For older Qdrant versions or default vector configuration
                actual_size = collection_info.config.params.size
            else:
                # Fallback to the embedding size
                actual_size = expected_size

            if actual_size != expected_size:
                logger.warning(f"Collection {collection_name} has vector size {actual_size}, but expected {expected_size}")
        except:
            # Create collection if it doesn't exist
            self.qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(size=len(embeddings[0]) if embeddings else self.embedding_service.get_embedding_size(), distance=models.Distance.COSINE)
            )
            logger.info(f"Created new Qdrant collection: {collection_name}")

        # Prepare points for insertion
        points = []
        chunk_ids = []

        # Get the current count of points to start from the next ID
        try:
            current_count = self.qdrant_client.count(collection_name=collection_name).count
        except:
            current_count = 0

        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            chunk_id = current_count + i + 1  # Use sequential integer IDs
            chunk_ids.append(str(chunk_id))  # Store as string for return

            points.append(
                PointStruct(
                    id=chunk_id,  # Use integer ID as required by Qdrant
                    vector=embedding,
                    payload={
                        "content": chunk,
                        "collection_name": collection_name,
                        "chunk_index": i,
                        "original_chunk_id": f"{collection_name}_chunk_{i}"  # Keep original ID in payload
                    }
                )
            )

        # Insert points into Qdrant
        self.qdrant_client.upsert(
            collection_name=collection_name,
            points=points
        )

        logger.info(f"Stored {len(chunk_ids)} chunks in collection {collection_name}")
        return chunk_ids

    async def retrieve_context(self, query: str, top_k: int = 5, collection_name: str = None) -> List[Dict]:
        """
        Retrieve relevant context from vector database based on query (Qdrant Cloud only - no fallback)
        Ensure retrieved chunks return: chunk_id, content, score/confidence, and source metadata
        """
        if collection_name is None:
            collection_name = settings.QDRANT_COLLECTION_NAME

        # Use Qdrant client - no fallback option
        # Verify the collection exists
        try:
            collection_info = self.qdrant_client.get_collection(collection_name)
            logger.debug(f"Collection {collection_name} exists with {collection_info.points_count} points")
        except Exception as e:
            logger.error(f"Collection {collection_name} does not exist: {e}")
            return []

        # Generate embedding for the query
        try:
            query_embedding = await self.embedding_service.generate_embedding(query)
        except Exception as e:
            logger.error(f"Failed to generate embedding for query: {e}")
            return []

        # Query in Qdrant (using the newer query_points method)
        try:
            from qdrant_client.http import models
            search_results = self.qdrant_client.query_points(
                collection_name=collection_name,
                query=query_embedding,
                limit=top_k,
                with_payload=True
            ).points
        except Exception as e:
            logger.error(f"Error searching in Qdrant: {e}")
            return []

        results = []
        for result in search_results:
            # Extract source metadata from the payload
            source_metadata = result.payload.get("source", result.payload.get("collection_name", collection_name))

            results.append({
                "chunk_id": str(result.id),  # Convert to string to ensure consistency
                "content": result.payload.get("content", ""),
                "score": result.score,  # Using 'score' instead of 'confidence' to match requirements
                "source": source_metadata
            })

        logger.info(f"Retrieved {len(results)} chunks from Qdrant collection {collection_name}")
        return results

    def _calculate_similarity(self, embedding1: List[float], embedding2: List[float]) -> float:
        """
        Calculate cosine similarity between two embeddings
        """
        # Simplified similarity calculation
        # In a real implementation, this would use proper cosine similarity
        dot_product = sum(a * b for a, b in zip(embedding1, embedding2))
        magnitude1 = sum(a * a for a in embedding1) ** 0.5
        magnitude2 = sum(b * b for b in embedding2) ** 0.5

        if magnitude1 == 0 or magnitude2 == 0:
            return 0.0

        return dot_product / (magnitude1 * magnitude2)

    async def generate_response(self, query: str, context: List[Dict], max_tokens: int = 500) -> str:
        """
        Generate response based on query and retrieved context using Gemini
        Enhanced to provide better context formatting and response quality
        """
        if not self.generative_model:
            # If no generative model is available, return a fallback response based on context
            logger.warning("No generative model available, returning context-based response")
            context_texts = [item["content"] for item in context if item.get("content", "").strip()]

            if not context_texts:
                return "The book does not provide details about this topic. No context is available and no generative model is configured."
            else:
                # Return a summary of the context as a fallback
                combined_context = " ".join(context_texts[:2])  # Take first 2 context items
                if len(combined_context) > 500:
                    combined_context = combined_context[:500] + "..."
                return f"Based on the book content: {combined_context}"

        # Prepare the context for the LLM with better formatting
        context_texts = [item["content"] for item in context if item.get("content", "").strip()]

        if not context_texts:
            # If no context is provided, respond with a specific message
            prompt = f"""You are a helpful AI assistant for a humanoid robotics book. Answer the user's question based on general knowledge.
If you cannot answer based on general knowledge, say "The book does not provide details about this topic."

Question: {query}

Please provide a helpful response."""
        else:
            # Format context with source information and relevance scores
            formatted_context = []
            for i, item in enumerate(context_texts):
                score = item.get("score", "N/A") if isinstance(item, dict) else "N/A"
                source = item.get("source", "Unknown") if isinstance(item, dict) else "Unknown"

                # If the item is just a string (from context_texts), we can't access score/source
                if isinstance(item, str):
                    formatted_context.append(f"Document {i+1}: {item}")
                else:
                    formatted_context.append(f"Document {i+1} (Relevance: {score:.2f}, Source: {source}): {item['content']}")

            context_str = "\n\n".join(formatted_context)

            prompt = f"""You are a helpful AI assistant for a humanoid robotics book. Use the following context to answer the user's question accurately and comprehensively.

Context:
{context_str}

Question: {query}

Instructions:
- Provide a detailed, accurate response based on the context
- If the context does not contain enough information, say "The book does not provide details about this topic."
- If the context is relevant but doesn't fully answer the question, provide what information is available and acknowledge limitations
- Maintain a professional, informative tone appropriate for technical content
- Include relevant technical details if present in the context

Response:"""

        try:
            # Generate response using Gemini with improved safety settings
            response = await self.generative_model.generate_content_async(
                prompt,
                generation_config={
                    "temperature": 0.7,
                    "max_output_tokens": max_tokens,
                    "candidate_count": 1,
                    "stop_sequences": ["Question:", "Context:"]
                },
                safety_settings=[
                    {
                        "category": "HARM_CATEGORY_DANGEROUS_CONTENT",
                        "threshold": "BLOCK_ONLY_HIGH"
                    },
                    {
                        "category": "HARM_CATEGORY_HATE_SPEECH",
                        "threshold": "BLOCK_ONLY_HIGH"
                    },
                    {
                        "category": "HARM_CATEGORY_HARASSMENT",
                        "threshold": "BLOCK_ONLY_HIGH"
                    },
                    {
                        "category": "HARM_CATEGORY_SEXUALLY_EXPLICIT",
                        "threshold": "BLOCK_ONLY_HIGH"
                    }
                ]
            )

            # Extract the text from the response
            if response.candidates and len(response.candidates) > 0:
                response_text = response.candidates[0].content.parts[0].text
                # Clean up any potential artifacts from the generation
                response_text = response_text.strip()

                # If the response contains the fallback message, return it as-is
                if "The book does not provide details about this topic." in response_text:
                    return "The book does not provide details about this topic."

                return response_text
            else:
                return "The book does not provide details about this topic."

        except Exception as e:
            error_msg = str(e).lower()
            # Check if it's a quota exceeded error
            if "quota" in error_msg or "exceeded" in error_msg or "429" in error_msg:
                logger.warning(f"Gemini API quota exceeded or rate limit hit: {e}")

                # Only try to rotate if we have multiple models configured
                if len(self.gemini_models) > 1 and await self._try_rotate_model():
                    logger.info("Switched to alternative Gemini model, retrying generation...")
                    # Retry the generation with the new model
                    return await self.generate_response(query, context, max_tokens)
                else:
                    logger.warning("Gemini API quota exceeded and no alternative models available")
                    # If no alternative models, return a helpful message
                    return "I've reached my response limit for now. Please try again later, or try rephrasing your question."
            # Check if it's a model not found error (like when a model doesn't exist)
            elif "404" in str(e) or "not found" in error_msg or "not supported" in error_msg:
                logger.error(f"Gemini model not available: {e}")

                # Only try to rotate if we have multiple models configured
                if len(self.gemini_models) > 1 and await self._try_rotate_model():
                    logger.info("Switched to alternative Gemini model, retrying generation...")
                    # Retry the generation with the new model
                    return await self.generate_response(query, context, max_tokens)
                else:
                    logger.warning("Gemini model not available and no alternative models")
                    # If no alternative models, return a helpful message
                    return "I'm currently unable to generate a response. Please try again later, or try rephrasing your question."
            else:
                logger.error(f"Error generating response with Gemini: {e}")
                # Fallback response
                return "The book does not provide details about this topic."

    async def generate_confidence_score(self, query: str, response: str, sources: List[Dict]) -> float:
        """
        Generate confidence score for the response based on source relevance
        Enhanced to consider multiple factors for better confidence estimation
        """
        if not sources:
            return 0.0

        # Calculate confidence based on multiple factors
        total_score = 0.0
        score_count = 0

        for source in sources:
            # Use the score field if available (from similarity calculations)
            if "score" in source:
                total_score += source["score"]
                score_count += 1
            # Use confidence field if available (for backward compatibility)
            elif "confidence" in source:
                total_score += source["confidence"]
                score_count += 1
            # Use a default score if no confidence information is available
            else:
                # If we have content, give it a moderate score
                if source.get("content", ""):
                    total_score += 0.5
                    score_count += 1

        # Calculate average confidence
        avg_confidence = total_score / score_count if score_count > 0 else 0.0

        # Adjust confidence based on response length (longer responses may indicate more confidence)
        if len(response) > 100:
            avg_confidence = min(avg_confidence * 1.1, 1.0)  # Slight boost for longer, detailed responses
        elif len(response) < 20:
            avg_confidence = max(avg_confidence * 0.8, 0.0)  # Reduce confidence for very short responses

        # Additional check: if the response is the fallback message, return very low confidence
        if "The book does not provide details about this topic." in response:
            return 0.1

        return min(avg_confidence, 1.0)  # Cap at 1.0

    async def _try_rotate_model(self) -> bool:
        """
        Rotate to the next available Gemini model when quota is exceeded or model is unavailable.
        Returns True if a new model was selected, False if all models are exhausted.
        """
        if not self.gemini_models or len(self.gemini_models) <= 1:
            return False

        # Move to the next model in rotation (circular)
        self.current_model_index = (self.current_model_index + 1) % len(self.gemini_models)

        # Select the new model
        self.generative_model = self.gemini_models[self.current_model_index]
        model_name = self.generative_model.model_name.split('/')[-1]
        logger.info(f"Switched to Gemini model: {model_name}")

        # Check if we're back to the original model (meaning we've tried all models)
        if self.current_model_index == 0 and len(self.gemini_models) > 1:
            logger.warning("All available Gemini models have been tried")
            return False

        return True

    async def process_query(self, query: str, mode: str = "full_book", selected_text: Optional[str] = None, top_k: int = 5) -> RAGResponse:
        """
        Process a query using the RAG pipeline
        Enhanced with better error handling, context management, and response quality
        """
        monitoring.log_user_action("system", "rag_query", {"query_length": len(query), "mode": mode, "top_k": top_k})

        start_time = __import__('time').time()

        try:
            # Determine context based on mode, but validate selected_text
            # If mode is selected_text but no valid selected_text provided, use full book mode
            if mode == "selected_text" and selected_text and selected_text.strip():
                # Use the enhanced selected_text mode that also searches the full database
                logger.info(f"Using selected_text mode for query: '{query[:50]}...' with selected text: '{selected_text[:100]}...'")
                context = await self.retrieve_context_from_text(query, selected_text)
            else:
                # Use full book content (stored in vector database)
                # Log which mode is being used and why the fallback occurred
                if mode == "selected_text" and (not selected_text or not selected_text.strip()):
                    logger.info(f"Fallback from selected_text to full_book mode for query: '{query[:50]}...' - no valid selected text provided")
                context = await self.retrieve_context(query, top_k)

            # Generate response
            response_text = await self.generate_response(query, context)

            # Prepare references - handle cases where sources might not have 'source' field
            references = []
            for item in context:
                if isinstance(item, dict) and "source" in item:
                    source = item["source"]
                    if source and source not in references:
                        references.append(source)
                elif isinstance(item, dict) and "content" in item:
                    # Use a portion of the content as reference if no source is available
                    content_preview = item["content"][:50] + "..." if len(item["content"]) > 50 else item["content"]
                    if content_preview and content_preview not in references:
                        references.append(content_preview)

            # Log performance and chunk retrieval details
            execution_time = __import__('time').time() - start_time
            monitoring.log_performance("rag_process_query", execution_time, {"status": "success", "chunks_retrieved": len(context)})

            # Log chunk retrieval details
            logger.info(f"Retrieved {len(context)} chunks for query: '{query[:50]}...'")
            if hasattr(self, 'generative_model') and self.generative_model:
                logger.info(f"Generated response using Gemini model")

            # Log the actual response length for quality assessment
            logger.debug(f"Generated response length: {len(response_text)} characters")

            return RAGResponse(
                response=response_text,
                sources=context,
                references=list(set(references))  # Remove duplicates
            )

        except ConnectionError as e:
            logger.error(f"Connection error during RAG query processing: {e}")
            monitoring.log_error(e, "Qdrant Cloud connection error")
            raise HTTPException(
                status_code=503,
                detail="Service temporarily unavailable due to Qdrant Cloud connection issues"
            )
        except Exception as e:
            error_msg = str(e).lower()
            # Check if it's a quota exceeded error and handle it with model rotation
            if "quota" in error_msg or "exceeded" in error_msg or "429" in error_msg:
                logger.warning(f"Gemini API quota exceeded in process_query: {e}")

                # Only try to rotate if we have multiple models configured
                if len(self.gemini_models) > 1 and await self._try_rotate_model():
                    logger.info("Switched to alternative Gemini model, retrying query...")
                    # Retry the entire query with the new model
                    return await self.process_query(query, mode, selected_text, top_k)
                else:
                    logger.warning("Gemini API quota exceeded and no alternative models available")
                    # If no alternative models, return a helpful response
                    return RAGResponse(
                        response="I've reached my response limit for now. Please try again later, or try rephrasing your question.",
                        sources=[],
                        references=[]
                    )
            # Check if it's a model not found error (like when a model doesn't exist)
            elif "404" in str(e) or "not found" in error_msg or "not supported" in error_msg:
                logger.error(f"Gemini model not available in process_query: {e}")

                # Only try to rotate if we have multiple models configured
                if len(self.gemini_models) > 1 and await self._try_rotate_model():
                    logger.info("Switched to alternative Gemini model, retrying query...")
                    # Retry the entire query with the new model
                    return await self.process_query(query, mode, selected_text, top_k)
                else:
                    logger.warning("Gemini model not available and no alternative models")
                    # If no alternative models, return a helpful response
                    return RAGResponse(
                        response="I'm currently unable to generate a response. Please try again later, or try rephrasing your question.",
                        sources=[],
                        references=[]
                    )
            else:
                logger.error(f"Error during RAG query processing: {e}")
                monitoring.log_error(e, "RAG query processing failed")
                # Return a more informative error response instead of raising exception
                return RAGResponse(
                    response="The book does not provide details about this topic.",
                    sources=[],
                    references=[]
                )

    async def retrieve_context_from_text(self, query: str, text: str) -> List[Dict]:
        """
        Retrieve context from provided text (for selected-text mode)
        Make sure selected_text mode: cleans the text, chunks the text, embeds the text,
        ranks chunks by similarity, returns them sorted, but still uses Gemini to generate the final answer.

        Now enhanced to also search in the full database for additional context.
        """
        # Clean the provided text
        cleaned_text = await self.clean_text(text)

        # Chunk the provided text
        chunks = await self.chunk_text(cleaned_text)

        # Embed the query for similarity matching
        try:
            query_embedding = await self.embedding_service.generate_embedding(query)
        except Exception as e:
            logger.error(f"Failed to generate embedding for query in selected text mode: {e}")
            # If embedding fails, fall back to just using full database context
            full_db_context = await self.retrieve_context(query, top_k=5)
            return full_db_context

        # Calculate relevance scores for each chunk
        selected_text_context = []
        for i, chunk in enumerate(chunks):
            try:
                # Generate embedding for the chunk
                chunk_embedding = await self.embedding_service.generate_embedding(chunk)

                # Calculate similarity between query and chunk
                similarity = self._calculate_similarity(query_embedding, chunk_embedding)

                selected_text_context.append({
                    "chunk_id": f"selected_text_chunk_{i}",
                    "content": chunk,
                    "score": similarity,  # Using 'score' instead of 'confidence' to match requirements
                    "source": "selected_text_input"  # Source metadata for selected text mode
                })
            except Exception as e:
                logger.warning(f"Failed to process chunk {i} in selected text mode: {e}")
                continue  # Skip this chunk and continue with others

        # Sort by score in descending order (rank chunks by similarity)
        selected_text_context.sort(key=lambda x: x["score"], reverse=True)

        # Also search in the full database for additional context
        full_db_context = await self.retrieve_context(query, top_k=3)  # Get 3 additional chunks from full database

        # Combine both contexts, prioritizing selected text context
        # But only include selected text context if it has good scores (above a threshold)
        MIN_RELEVANCE_THRESHOLD = 0.1  # Minimum similarity threshold
        filtered_selected_context = [item for item in selected_text_context if item["score"] > MIN_RELEVANCE_THRESHOLD]

        # Combine contexts: selected text context first, then full DB context
        combined_context = filtered_selected_context + full_db_context

        # If no relevant selected text context, just use full DB context
        if not combined_context:
            combined_context = full_db_context

        # Limit to top 5 chunks total
        return combined_context[:5]

    async def store_selected_text_chunks(self, text: str, user_id: str, collection_name: str = None) -> List[str]:
        """
        Store selected text chunks in a separate Qdrant collection for future reference
        """
        if collection_name is None:
            collection_name = f"selected_text_{user_id}"

        # Clean and chunk the provided text
        cleaned_text = await self.clean_text(text)
        chunks = await self.chunk_text(cleaned_text)

        # Generate embeddings for the chunks
        embeddings = await self.embed_chunks(chunks)

        # Store in the selected text collection
        chunk_ids = await self.store_chunks(chunks, embeddings, collection_name)

        return chunk_ids

    async def ingest_content(self, content: str, collection_name: str = None, progress_tracker=None) -> Dict:
        """
        Ingest content into the RAG system (clean → chunk → embed → store flow)
        """
        if collection_name is None:
            collection_name = settings.QDRANT_COLLECTION_NAME

        try:
            # Clean the content
            if progress_tracker:
                progress_tracker.update_progress(status="cleaning", progress_percentage=10)

            cleaned_content = await self.clean_text(content)

            # Chunk the content
            if progress_tracker:
                progress_tracker.update_progress(status="chunking", progress_percentage=20)

            chunks = await self.chunk_text(cleaned_content)

            # Generate embeddings for chunks
            if progress_tracker:
                progress_tracker.update_progress(status="embedding", progress_percentage=30)

            embeddings = await self.embed_chunks(chunks)

            # Update progress during embedding (more granular but less frequent updates)
            if progress_tracker and len(chunks) > 0:
                # Update progress periodically during embedding (every 10% of chunks)
                update_interval = max(1, len(chunks) // 10)  # Update every 10% of chunks
                for i, chunk in enumerate(chunks):
                    if i % update_interval == 0 or i == len(chunks) - 1:  # Update every interval or at the end
                        progress = 30 + int((i / len(chunks)) * 40)  # 30% to 70% for embedding
                        progress_tracker.update_progress(
                            status="embedding",
                            progress_percentage=min(progress, 70)
                        )

            # Store in vector database
            if progress_tracker:
                progress_tracker.update_progress(status="storing", progress_percentage=70)

            chunk_ids = await self.store_chunks(chunks, embeddings, collection_name)

            # Final update
            if progress_tracker:
                progress_tracker.update_progress(
                    status="completed",
                    total_chunks=len(chunks),
                    progress_percentage=100
                )

            return {
                "chunks_processed": len(chunks),
                "chunk_ids": chunk_ids,
                "collection_name": collection_name
            }
        except Exception as e:
            if progress_tracker:
                progress_tracker.update_progress(
                    status="failed",
                    error_message=str(e),
                    progress_percentage=0
                )
            logger.error(f"Error during content ingestion: {e}")
            monitoring.log_error(e, "Content ingestion failed")
            raise