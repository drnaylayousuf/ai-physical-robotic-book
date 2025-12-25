# Integrated RAG Chatbot for Physical AI and Humanoid Robotics Book

This repository contains both the "Physical AI & Humanoid Robotics: Building Embodied Intelligent Systems" book and an integrated RAG chatbot system that allows users to ask questions about the book content.

## About the Book

This comprehensive textbook teaches Physical AI—AI systems that interact with the physical world—centered around Humanoid Robotics, ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action systems.

### Topics Covered

- Physical AI & Embodied Intelligence
- ROS 2 Fundamentals
- Gazebo & Unity Simulation
- NVIDIA Isaac Platform
- Bipedal Control & Humanoid Mechanics
- Vision-Language-Action Robotics (LLM-Driven Robots)
- Conversational Robotics (Speech → Action)
- Capstone: Autonomous Humanoid Project
- Lab Setup, Hardware Requirements, and Cloud Architecture

## RAG Chatbot System

The integrated RAG (Retrieval-Augmented Generation) chatbot allows users to ask questions about the book content and receive AI-generated responses based on the book's text. The system uses FastAPI backend with Qdrant vector database and Neon Postgres for metadata, integrated with ChatKit-JS frontend.

### Features

- Interactive chat interface for asking questions about book content
- Two RAG modes:
  - Full-book RAG: Answers based on entire book content
  - Selected-text RAG: Answers based only on user-selected text
- Source attribution with chunk IDs and paragraph references
- User authentication and role-based access control
- Streaming responses
- Admin functionality for content ingestion

## Prerequisites

- Python 3.11+
- Node.js 18+ (for frontend development)
- Docker and Docker Compose
- Access to Gemini API (for embeddings and generation)
- Qdrant Cloud account (or local Qdrant instance)
- Neon Postgres account (or local Postgres instance)

## Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd humanoid-robotics-book
   ```

2. Install backend dependencies:
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r ../requirements.txt
   ```

3. Create and configure environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your API keys and configuration
   ```

4. Start the services using Docker Compose:
   ```bash
   docker-compose up --build
   ```

## Development

### Book Content
To work with the book content:

```bash
npm install
npm start
```

This command starts a local development server for the Docusaurus book.

### Backend
The backend is built with FastAPI and runs on port 8000. To run in development mode:

```bash
cd backend
uvicorn main:app --reload
```

### Frontend
The frontend files are in the `frontend/` directory and can be served with a simple HTTP server.

### CORS Configuration
This application uses CORS (Cross-Origin Resource Sharing) to enable communication between the frontend and backend:

- **Backend**: Runs on `http://localhost:8000`
- **Frontend**: Runs on `http://localhost:3000`
- **CORS Configuration**: The backend is configured to allow requests from `http://localhost:3000`

If you change the frontend port, you'll need to update the CORS configuration in `backend/main.py` accordingly.

## API Endpoints

- `POST /ask` - Ask questions about the book content
- `POST /ingest` - Ingest book content (admin only)
- `GET /health` - Health check
- `GET /metadata` - Get book metadata
- `POST /auth/register` - User registration
- `POST /auth/login` - User login
- `GET /auth/profile` - Get user profile

## Architecture

The system follows a microservices architecture with:
- FastAPI backend for API endpoints
- Qdrant vector database for content embeddings
- Postgres database for user data and metadata
- Frontend with ChatKit-JS integration

## Contributing

This book was generated using Spec-Kit Plus methodology with Claude Code as the AI assistant. Contributions are welcome!

## License

This project is licensed under the MIT License.

## Railway Deployment

This project is optimized for Railway deployment with:
- Dockerfile optimized to stay under 4GB limit
- Environment variables configured for cloud deployment
- Qdrant Cloud integration for vector storage
- Cohere embeddings and Gemini LLM for responses