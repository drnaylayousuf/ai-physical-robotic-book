# Data Model: CORS Configuration for FastAPI

## CORS Configuration Entity

### Attributes
- **origin**: String representing the allowed origin URL (e.g., "http://localhost:3000")
- **credentials_allowed**: Boolean indicating whether credentials can be sent with requests
- **allowed_methods**: List of HTTP methods that are permitted (GET, POST, OPTIONS, etc.)
- **allowed_headers**: List of HTTP headers that are permitted
- **max_age**: Integer representing the maximum age for preflight requests (optional)

### Relationships
- **FastAPI Application**: The CORS configuration is applied to a specific FastAPI application instance
- **API Endpoints**: All API endpoints under the FastAPI application inherit the CORS configuration

### Validation Rules
- Origins must be valid URLs with protocol, host, and optional port
- Methods must be valid HTTP methods
- Headers must be valid HTTP header names
- If credentials are allowed, wildcard origins are not permitted (security requirement)

### State Transitions
- **Development Configuration**: Allows localhost origins for development
- **Production Configuration**: Restricts to specific production domains only

## API Endpoint Entity

### Attributes
- **path**: String representing the API endpoint path (e.g., "/api/ask")
- **method**: HTTP method (GET, POST, PUT, DELETE, etc.)
- **response_type**: Expected response format
- **authentication_required**: Boolean indicating if authentication is needed

### Relationships
- **CORS Configuration**: Each endpoint inherits the CORS configuration from the parent FastAPI application