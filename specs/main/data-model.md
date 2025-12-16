# Data Model: Integrated RAG Chatbot for Physical AI and Humanoid Robotics Book

## Entities

### User
- **id**: SERIAL PRIMARY KEY
- **username**: VARCHAR(255) UNIQUE NOT NULL
- **email**: VARCHAR(255) UNIQUE NOT NULL
- **password_hash**: VARCHAR(255) NOT NULL
- **role**: VARCHAR(50) DEFAULT 'user' (values: 'admin', 'moderator', 'user', 'guest')
- **created_at**: TIMESTAMP DEFAULT CURRENT_TIMESTAMP
- **updated_at**: TIMESTAMP DEFAULT CURRENT_TIMESTAMP
- **Relationships**: One-to-many with UserQuery, UserSelectedText

### UserQuery
- **id**: SERIAL PRIMARY KEY
- **user_id**: INTEGER REFERENCES users(id)
- **question**: TEXT
- **response**: TEXT
- **sources**: JSONB
- **timestamp**: TIMESTAMP DEFAULT CURRENT_TIMESTAMP
- **Relationships**: Many-to-one with User

### UserSelectedText
- **id**: SERIAL PRIMARY KEY
- **user_id**: INTEGER REFERENCES users(id)
- **selected_text**: TEXT
- **context_metadata**: JSONB
- **timestamp**: TIMESTAMP DEFAULT CURRENT_TIMESTAMP
- **Relationships**: Many-to-one with User

### ChapterMetadata
- **id**: SERIAL PRIMARY KEY
- **chapter_title**: VARCHAR(255)
- **file_path**: VARCHAR(500)
- **content_summary**: TEXT
- **paragraph_count**: INTEGER
- **created_at**: TIMESTAMP DEFAULT CURRENT_TIMESTAMP
- **Relationships**: Used for book content organization

## Validation Rules

### User Entity
- Username and email must be unique
- Password must be properly hashed
- Role must be one of the allowed values
- Email format must be valid

### UserQuery Entity
- user_id must reference an existing user
- question and response must be non-empty
- sources must be valid JSON

### UserSelectedText Entity
- user_id must reference an existing user
- selected_text must be non-empty
- context_metadata must be valid JSON

### ChapterMetadata Entity
- file_path must be a valid path
- paragraph_count must be non-negative

## State Transitions

### User Authentication Flow
1. Guest (no account) → Registered User (account created)
2. Registered User → Active User (authenticated session)
3. Active User → Different Role (admin, moderator privileges granted)

### Query Processing Flow
1. Question submitted → Processing → Response generated → Stored in database
2. With source attribution and confidence scoring

## Indexes

### User Table
- Index on username (for authentication)
- Index on email (for account recovery)

### UserQuery Table
- Index on user_id (for user history queries)
- Index on timestamp (for time-based queries)

### UserSelectedText Table
- Index on user_id (for user history queries)
- Index on timestamp (for time-based queries)

### ChapterMetadata Table
- Index on chapter_title (for search)
- Index on file_path (for content lookup)