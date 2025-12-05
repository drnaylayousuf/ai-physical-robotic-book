## Data Model: Physical AI & Humanoid Robotics Book

This section outlines the key entities and their relationships within the Docusaurus book for the "Physical AI & Humanoid Robotics" course.

### 1. Course
*   **Name**: Represents the overall "Physical AI & Humanoid Robotics" course.
*   **Fields**:
    *   `title`: "Physical AI & Humanoid Robotics"
    *   `goal`: "Students will build a simulated autonomous humanoid that follows voice commands using ROS 2, Gazebo, Unity, NVIDIA Isaac Sim & Isaac ROS, Vision-Language-Action (VLA), Whisper + LLM planning, Digital twins + bipedal locomotion."
    *   `modules`: (Relationship: One-to-Many with Module)

### 2. Module
*   **Name**: A major section of the course, corresponding to a top-level directory under `/docs`.
*   **Fields**:
    *   `id`: Unique identifier (e.g., `ros2`, `digital-twin`)
    *   `title`: Display name (e.g., "ROS 2", "Digital Twin (Gazebo & Unity)")
    *   `description`: Brief overview of the module's content.
    *   `chapters`: (Relationship: One-to-Many with Chapter/Subchapter)
*   **Validation Rules**:
    *   Each module must have a unique `id`.
    *   Each module `id` must correspond to a folder name under `/docs`.

### 3. Chapter/Subchapter
*   **Name**: Individual units of content within a module, corresponding to Markdown (`.mdx`) files.
*   **Fields**:
    *   `id`: Unique identifier within its module.
    *   `title`: Display name for the sidebar.
    *   `file_path`: Relative path to the `.mdx` file.
    *   `content`: Markdown/MDX content of the chapter.
    *   `diagrams`: (Relationship: One-to-Many with Diagram)
    *   `code_examples`: (Relationship: One-to-Many with Code Example)
    *   `labs_exercises`: Boolean indicating presence of labs/exercises.
    *   `parent_module_id`: (Relationship: Many-to-One with Module)
*   **Validation Rules**:
    *   Each chapter/subchapter must have a unique `id` within its module.
    *   File path must exist and point to an `.mdx` file.
    *   Content must adhere to MDX formatting.

### 4. Diagram
*   **Name**: Visual representations (Mermaid) of architectures, pipelines, or robot kinematics.
*   **Fields**:
    *   `type`: "Mermaid"
    *   `code`: Mermaid diagram syntax.
    *   `caption`: Description of the diagram.
    *   `parent_chapter_id`: (Relationship: Many-to-One with Chapter/Subchapter)

### 5. Code Example
*   **Name**: Snippets demonstrating concepts.
*   **Fields**:
    *   `language`: Programming language (e.g., "Python", "C++", "ROS 2 CLI", "Isaac scripts", "Launch files").
    *   `code`: Code snippet content.
    *   `description`: Explanation of the code.
    *   `parent_chapter_id`: (Relationship: Many-to-One with Chapter/Subchapter)

### 6. Hardware
*   **Name**: Recommended physical components for the lab, detailed in the `/hardware` section.
*   **Fields**:
    *   `component_name`: Name of the hardware component (e.g., "High-Performance Workstation", "Jetson Orin Kit", "Intel RealSense D435i").
    *   `specs`: Technical specifications.
    *   `usage_context`: How it's used in the course.

### 7. Glossary Term
*   **Name**: Definitions of key terminology, detailed in a dedicated `/glossary` section.
*   **Fields**:
    *   `term`: The word or phrase being defined.
    *   `definition`: The explanation of the term.
