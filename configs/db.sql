-- Database schema for Integrated RAG Chatbot for Physical AI and Humanoid Robotics Book

-- User authentication and profiles
CREATE TABLE users (
    id SERIAL PRIMARY KEY,
    username VARCHAR(255) UNIQUE NOT NULL,
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    role VARCHAR(50) DEFAULT 'user', -- Possible values: 'admin', 'moderator', 'user', 'guest'
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- User queries history
CREATE TABLE user_queries (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id),
    question TEXT,
    response TEXT,
    sources JSONB,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- User selected text
CREATE TABLE user_selected_text (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id),
    selected_text TEXT,
    context_metadata JSONB,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Chapter metadata
CREATE TABLE chapter_metadata (
    id SERIAL PRIMARY KEY,
    chapter_title VARCHAR(255),
    file_path VARCHAR(500),
    content_summary TEXT,
    paragraph_count INTEGER,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create indexes for better performance
CREATE INDEX idx_users_username ON users(username);
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_user_queries_user_id ON user_queries(user_id);
CREATE INDEX idx_user_queries_timestamp ON user_queries(timestamp);
CREATE INDEX idx_user_selected_text_user_id ON user_selected_text(user_id);
CREATE INDEX idx_user_selected_text_timestamp ON user_selected_text(timestamp);
CREATE INDEX idx_chapter_metadata_title ON chapter_metadata(chapter_title);
CREATE INDEX idx_chapter_metadata_path ON chapter_metadata(file_path);

-- Insert default admin user (password: admin123)
-- Note: In production, this should be handled through a secure admin setup process
-- The password should be properly hashed before insertion
-- INSERT INTO users (username, email, password_hash, role)
-- VALUES ('admin', 'admin@example.com', '$2b$12$...', 'admin');