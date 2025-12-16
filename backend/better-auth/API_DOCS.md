# Better Auth API Endpoints Documentation

## Authentication Endpoints

The following API endpoints are provided by Better Auth for user authentication:

### POST `/api/auth/signin`
- **Purpose**: Authenticate existing user
- **Request Body**:
  ```json
  {
    "email": "user@example.com",
    "password": "user_password"
  }
  ```
- **Response**:
  ```json
  {
    "user": { ...user_data },
    "session": { ...session_data }
  }
  ```

### POST `/api/auth/signup`
- **Purpose**: Register new user
- **Request Body**:
  ```json
  {
    "email": "user@example.com",
    "password": "user_password",
    "name": "User Name" (optional)
  }
  ```
- **Response**:
  ```json
  {
    "user": { ...user_data },
    "session": { ...session_data }
  }
  ```

### POST `/api/auth/onboarding`
- **Purpose**: Complete user profile during onboarding
- **Request Body**:
  ```json
  {
    "userId": "user_id",
    "pythonProficiency": "BEGINNER|INTERMEDIATE|EXPERT",
    "operatingSystem": "WINDOWS|MACOS|LINUX|OTHER",
    "preferredEnvironment": "LOCAL_MACHINE|CLOUD_ENVIRONMENT"
  }
  ```
- **Response**:
  ```json
  {
    "success": true,
    "profile": { ...profile_data },
    "redirectTo": "/dashboard"
  }
  ```

### GET `/api/auth/me`
- **Purpose**: Get current user profile
- **Response**:
  ```json
  {
    "user": { ...user_data },
    "profile": { ...profile_data },
    "onboardingComplete": true|false
  }
  ```

## Social Authentication

The system supports Google and GitHub social authentication providers.

## Session Management

Sessions are managed automatically by Better Auth with configurable expiration times.