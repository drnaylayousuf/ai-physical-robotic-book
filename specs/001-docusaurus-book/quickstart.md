## Quickstart: Running the Physical AI & Humanoid Robotics Book Locally

This guide provides instructions to quickly set up and run the "Physical AI & Humanoid Robotics" Docusaurus book on your local machine. This assumes you already have a Docusaurus project initialized and its dependencies installed.

### Prerequisites

*   **Node.js**: Ensure you have Node.js (version 18 or higher recommended) and npm (or Yarn) installed.
*   **Docusaurus Dependencies**: The Docusaurus project dependencies (`node_modules`) should already be installed. If not, navigate to the project root and run `npm install` (or `yarn install`).

### 1. Navigate to the Project Root

Open your terminal or command prompt and change the directory to the root of this repository:

```bash
cd /path/to/humanoid-robotics-book
```

### 2. Start the Docusaurus Development Server

Once in the project root, start the local development server:

```bash
npm start
# or if you use Yarn
yarn start
```

This command will:

*   Build the static assets for the Docusaurus site.
*   Launch a local development server, typically accessible at `http://localhost:3000`.
*   Open your default web browser to the site.
*   Enable hot-reloading, so any changes you make to the Markdown content or configuration files will automatically update in your browser.

### 3. Access the Book

Open your web browser and navigate to `http://localhost:3000` (or the address provided in your terminal output).

You should now see the "Physical AI & Humanoid Robotics" book with its full navigation sidebar and content.

### 4. Making Changes

*   **Content**: Edit the `.mdx` files located under the `/docs` directory to modify chapter content.
*   **Navigation**: Update `sidebars.js` in the project root to change the book's navigation structure.
*   **Configuration**: Modify `docusaurus.config.js` for site-wide settings, themes, and plugins.

### 5. Building for Production (Optional)

To build a static version of the book for deployment, run:

```bash
npm run build
# or
yarn build
```

This will generate static HTML, CSS, and JavaScript files in the `/build` directory, which can then be served by any static file host.

---

**Note**: This project assumes an existing Docusaurus setup. No Docusaurus initialization or dependency reinstallation steps are required.