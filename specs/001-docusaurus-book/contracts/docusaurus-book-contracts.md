## Docusaurus Book Contracts

This document outlines the "contracts" or agreements for how content and configuration files will interact within the Docusaurus project to ensure a consistent and navigable technical book.

### 1. Content File Structure Contract
*   **Location**: All Markdown content files (`.mdx`) for chapters and subchapters MUST reside within the `/docs` directory and its subfolders (e.g., `/docs/ros2`, `/docs/humanoid`).
*   **Module Directories**: Each major module (e.g., `intro`, `physical-ai`, `ros2`) MUST have its own dedicated subdirectory directly under `/docs`.
*   **Chapter Files**: Each chapter/subchapter within a module will be represented by an `.mdx` file.
*   **Index Files**: Each module directory MAY contain an `index.mdx` file to serve as the module's overview page.
*   **Naming Convention**: File and folder names should be lowercase and hyphen-separated (kebab-case) for consistency and URL friendliness.

### 2. Sidebar Navigation Contract (`sidebars.js`)
*   **Source**: The `sidebars.js` file MUST be the single source of truth for the entire book's navigation structure.
*   **Structure**: The `sidebars.js` file will define categories for each top-level module, and within those categories, individual doc entries for each chapter/subchapter.
*   **Categorization**: Modules will be organized into `type: 'category'` entries, with `label` matching the module title and `items` containing the chapters.
*   **Doc Entries**: Each chapter will be a `type: 'doc'` entry, using the `id` of the Markdown file (derived from its filename without the extension).
*   **Linking**: All internal links within Markdown files MUST use relative paths that correspond to the `id`s defined in `sidebars.js` (e.g., `[ROS 2 Introduction](/docs/ros2/intro)`).

### 3. Docusaurus Configuration Contract (`docusaurus.config.js`)
*   **Site Metadata**: The `title` and `tagline` in `docusaurus.config.js` MUST reflect the book's title and purpose.
*   **Navbar Links**: The `navbar.items` array MUST include primary links to the documentation (`docs`), and potentially other relevant sections (e.g., external resources).
*   **Plugins**: The `@docusaurus/preset-classic` plugin MUST be configured to enable MDX, Mermaid, and KaTeX support. This includes setting up `remarkPlugins` and `rehypePlugins` as needed.
*   **Syntax Highlighting**: Prism syntax highlighting MUST be configured for relevant code languages (Python, C++, XML/URDF, YAML, JavaScript, TypeScript, Bash).

### 4. Content Formatting Contract
*   **Markdown/MDX**: All content MUST use standard Markdown and MDX syntax for text, headings, lists, tables, and embedding React components.
*   **Code Blocks**: Code examples MUST be enclosed in fenced code blocks with appropriate language identifiers for syntax highlighting (e.g., ````python`, ````cpp`, ````mermaid`).
*   **Mermaid Diagrams**: Mermaid diagrams MUST be embedded using the ````mermaid` fence.
*   **KaTeX Math**: Mathematical expressions MUST use KaTeX syntax, typically enclosed within `$` (inline) or `$$` (block) delimiters.

### 5. Interlinking Contract
*   **Relative Paths**: Internal links between chapters and modules MUST use relative paths (e.g., `../ros2/nodes` or `/docs/ros2/nodes`).
*   **Image Paths**: Images and other assets MUST use relative paths within the `static/img` directory or directly within the chapter's folder if appropriate.

These contracts ensure that the generated content is well-structured, easily navigable, and consistently presented within the Docusaurus framework, providing a high-quality learning experience for the students.