# Production Deployment Guide

## Overview

This document provides instructions for deploying the Integrated RAG Chatbot for Physical AI and Humanoid Robotics Book to production environments. The application consists of a backend API, frontend UI, vector database, and relational database that need to be deployed and configured properly.

## Deployment Architecture

The application follows a microservices architecture with the following components:

- **Backend API**: FastAPI application (deployed to Railway or Render)
- **Frontend UI**: Static files (deployed to Vercel)
- **Vector Database**: Qdrant Cloud (managed service)
- **Relational Database**: Neon Postgres (managed service)

## Prerequisites

Before deploying, ensure you have:

1. **Qdrant Cloud Account**: For vector database storage
2. **Neon Postgres Account**: For metadata storage
3. **API Keys**: Gemini API key for LLM functionality
4. **Domain Names**: For backend and frontend applications
5. **SSL Certificates**: For secure communication (usually handled by deployment platforms)

## Backend Deployment (Railway/Render)

### Railway Deployment

1. **Prepare the application**:
   - Ensure your `Dockerfile` is properly configured
   - Verify all environment variables are defined in the Railway dashboard

2. **Deploy to Railway**:
   - Connect your GitHub repository to Railway
   - Create a new project and link it to your repository
   - Configure the following environment variables in Railway:
     ```
     DATABASE_URL=your_neon_postgres_connection_string
     QDRANT_URL=your_qdrant_cloud_url
     QDRANT_API_KEY=your_qdrant_api_key
     QDRANT_COLLECTION_NAME=book_chunks
     GEMINI_API_KEY=your_gemini_api_key
     GEMINI_MODEL=gemini-2.5-flash
     SECRET_KEY=your_secure_secret_key
     ALGORITHM=HS256
     ACCESS_TOKEN_EXPIRE_MINUTES=30
     BOOK_CONTENT_PATH=./doc
     CHUNK_SIZE=512
     CHUNK_OVERLAP=64
     MAX_CONTEXT_LENGTH=2048
     ```

3. **Configure the service**:
   - Set the build command: `pip install -r requirements.txt`
   - Set the start command: `uvicorn backend.main:app --host 0.0.0.0 --port $PORT`
   - Allocate appropriate resources (recommended: 1GB RAM for initial deployment)

### Render Deployment

1. **Create a Web Service** on Render
2. **Connect your GitHub repository**
3. **Configure the build**:
   - Environment: Python
   - Build Command: `pip install -r requirements.txt`
   - Start Command: `uvicorn backend.main:app --host 0.0.0.0 --port $PORT`
4. **Set environment variables** (same as Railway above)
5. **Configure resources** (recommended: Starter plan or higher)

## Frontend Deployment (Vercel)

1. **Prepare the frontend**:
   - Ensure all frontend files are in the `frontend/` directory
   - Update API endpoints in frontend code to point to your deployed backend

2. **Deploy to Vercel**:
   - Go to vercel.com and create an account
   - Import your GitHub repository
   - Configure the project:
     - Framework: Other (since it's static files)
     - Build Command: `echo "Build completed"`
     - Output Directory: `frontend/`
   - Set environment variables if needed (for API endpoints)

3. **Configure custom domain** (optional):
   - Add your custom domain in the Vercel dashboard
   - Update DNS records as instructed

## Database Setup

### Neon Postgres Setup

1. **Create a Neon project**:
   - Sign up at neon.tech
   - Create a new project
   - Note the connection string

2. **Run database migrations**:
   - After deploying the backend, run migrations to set up tables:
   ```bash
   alembic upgrade head
   ```

3. **Configure connection pooling**:
   - Adjust connection pool settings based on expected load
   - Monitor connection usage and scale as needed

### Qdrant Setup

1. **Create a Qdrant Cloud collection**:
   - Sign up at qdrant.tech
   - Create a new cluster
   - Create a collection named `book_chunks`
   - Set vector dimensions based on your embedding model (typically 768 for Gemini embeddings)

2. **Configure collection settings**:
   - Set up appropriate indexing for performance
   - Configure backup and recovery options

## Configuration Management

### Environment Variables

All sensitive configuration should be managed through environment variables:

**Required Variables**:
- `DATABASE_URL`: Neon Postgres connection string
- `QDRANT_URL`: Qdrant Cloud endpoint
- `QDRANT_API_KEY`: Qdrant API key
- `GEMINI_API_KEY`: Gemini API key
- `SECRET_KEY`: JWT secret key (use a strong random string)

**Optional Variables**:
- `QDRANT_COLLECTION_NAME`: Default is `book_chunks`
- `GEMINI_MODEL`: Default is `gemini-2.5-flash`
- `CHUNK_SIZE`: Default is `512`
- `CHUNK_OVERLAP`: Default is `64`
- `MAX_CONTEXT_LENGTH`: Default is `2048`
- `ACCESS_TOKEN_EXPIRE_MINUTES`: Default is `30`

### Secrets Management

1. **Generate a strong secret key** for JWT tokens:
   ```bash
   python -c 'import secrets; print(secrets.token_urlsafe(32))'
   ```

2. **Store secrets securely** in your deployment platform's secret management system

## Ingestion Process

After deploying and configuring the databases:

1. **Prepare book content** in the `doc/` directory
2. **Call the ingestion endpoint** to process the content:
   ```bash
   curl -X POST https://your-backend-domain.com/ingest \
        -H "Authorization: Bearer your-admin-token"
   ```

3. **Monitor the ingestion process** for progress and errors

## Monitoring and Observability

### Health Checks

The application provides a health check endpoint at `/health` that returns:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-09T10:00:00Z"
}
```

### Logging

- Configure your deployment platform to capture application logs
- Set up log aggregation for analysis
- Monitor error logs for issues

### Performance Monitoring

- Monitor response times for the `/ask` endpoint
- Track database query performance
- Monitor external API call latencies (Gemini, Qdrant)

## Scaling Guidelines

### Horizontal Scaling

- The backend is designed to scale horizontally
- Ensure session data is stored in a shared database, not in memory
- Configure load balancing appropriately

### Database Scaling

- Monitor database connection usage and scale accordingly
- Consider read replicas for high-traffic scenarios
- Optimize database indexes based on query patterns

### Vector Database Scaling

- Monitor Qdrant performance metrics
- Adjust cluster size based on query volume
- Consider collection sharding for very large datasets

## Backup and Recovery

### Database Backups

- Enable automated backups in Neon Postgres
- Regularly test backup restoration procedures
- Store backups in geographically distributed locations

### Vector Database Backups

- Configure Qdrant backup procedures
- Export important data periodically
- Test recovery procedures regularly

## Security Considerations

### API Security

- Use HTTPS for all API endpoints
- Implement rate limiting to prevent abuse
- Validate and sanitize all inputs
- Use JWT tokens with appropriate expiration times

### Data Security

- Encrypt data in transit using TLS
- Consider encrypting sensitive data at rest
- Implement proper access controls
- Regular security audits

## Troubleshooting

### Common Issues

1. **Connection Timeouts**: Check network connectivity and firewall rules
2. **Database Connection Issues**: Verify connection strings and credentials
3. **API Rate Limits**: Monitor external API usage and implement caching
4. **Memory Issues**: Monitor resource usage and scale appropriately

### Debugging Tips

- Enable detailed logging during troubleshooting
- Use deployment platform's monitoring tools
- Check external service status pages
- Review application logs systematically

## Rollback Procedures

### Automated Rollback

- Maintain previous versions in your deployment platform
- Use platform features for quick rollback to previous versions
- Test rollback procedures regularly

### Manual Rollback

1. Deploy the previous version of the application
2. Verify database compatibility
3. Test functionality thoroughly
4. Monitor for issues after rollback

## Maintenance Schedule

### Regular Maintenance

- Monitor application performance weekly
- Review logs for unusual patterns
- Update dependencies regularly
- Test backup and recovery procedures monthly

### Updates

- Deploy minor updates weekly
- Plan major updates with proper testing
- Schedule maintenance windows for major updates
- Communicate planned downtime to users