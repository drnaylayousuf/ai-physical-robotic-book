# Data Model: User Onboarding and Authentication

## User Entity
- **id**: string (primary key, unique)
- **email**: string (unique, required)
- **name**: string (optional)
- **emailVerified**: datetime (nullable)
- **createdAt**: datetime (default: now)
- **updatedAt**: datetime (default: now)

## UserProfile Entity
- **id**: string (primary key, unique)
- **userId**: string (foreign key to User.id, unique)
- **pythonProficiency**: enum (values: "BEGINNER", "INTERMEDIATE", "EXPERT", required)
- **operatingSystem**: enum (values: "WINDOWS", "MACOS", "LINUX", "OTHER", required)
- **preferredEnvironment**: enum (values: "LOCAL_MACHINE", "CLOUD_ENVIRONMENT", required)
- **onboardingComplete**: boolean (default: false)
- **createdAt**: datetime (default: now)
- **updatedAt**: datetime (default: now)

## Relationships
- User (1) → UserProfile (0..1): One user has zero or one profile
- UserProfile (1) → User (1): Each profile belongs to exactly one user

## Validation Rules
- UserProfile.pythonProficiency must be one of the defined enum values
- UserProfile.operatingSystem must be one of the defined enum values
- UserProfile.preferredEnvironment must be one of the defined enum values
- UserProfile.userId must reference an existing User record