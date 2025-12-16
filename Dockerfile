# Multi-stage build for Docusaurus site
FROM node:20 AS builder

WORKDIR /app

# Copy only package files first for better caching
COPY book-source/package*.json ./

# Install dependencies
RUN npm ci --only=production

# Copy the rest of the source code
COPY book-source/ ./

# Build the Docusaurus site
RUN npm run build

# Production stage - serve the static site
FROM nginx:alpine

# Copy built site from builder stage
COPY --from=builder /app/build /usr/share/nginx/html

# Copy custom nginx configuration to handle client-side routing
COPY nginx.conf /etc/nginx/conf.d/default.conf

EXPOSE 80

CMD ["nginx", "-g", "daemon off;"]