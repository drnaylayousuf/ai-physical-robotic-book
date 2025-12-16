# API Contracts: User Onboarding and Authentication

## Authentication Endpoints

### POST /api/auth/signup
- **Purpose**: Register new user and initiate onboarding
- **Request**:
  - email: string
  - password: string
- **Response**:
  - success: boolean
  - user: User object
  - requiresOnboarding: boolean
- **Error Response**:
  - error: string
  - code: string

### POST /api/auth/signin
- **Purpose**: Authenticate existing user
- **Request**:
  - email: string
  - password: string
- **Response**:
  - success: boolean
  - user: User object
  - requiresOnboarding: boolean
  - redirectTo: string (onboarding page if incomplete)
- **Error Response**:
  - error: string
  - code: string

### POST /api/auth/onboarding
- **Purpose**: Complete user profile during onboarding
- **Request**:
  - pythonProficiency: string (enum)
  - operatingSystem: string (enum)
  - preferredEnvironment: string (enum)
- **Response**:
  - success: boolean
  - profile: UserProfile object
  - redirectTo: string (main content)
- **Error Response**:
  - error: string
  - code: string

### GET /api/auth/me
- **Purpose**: Get current user profile
- **Response**:
  - user: User object
  - profile: UserProfile object (nullable)
  - onboardingComplete: boolean
- **Error Response**:
  - error: string
  - code: string