# Quickstart Guide: User Onboarding and Authentication

## Setting Up Authentication

### 1. Install Dependencies
```bash
npm install better-auth @better-auth/node
npm install @neondatabase/serverless prisma
npx prisma generate
```

### 2. Environment Variables
```env
# Better-Auth
BETTER_AUTH_SECRET="your-secret-key"
BETTER_AUTH_URL="http://localhost:3000"

# Neon Database
DATABASE_URL="your-neon-database-url"
```

### 3. Initialize Better-Auth
```javascript
// lib/auth.ts
import { betterAuth } from "better-auth";
import { neon } from "@neondatabase/serverless";

export const auth = betterAuth({
  database: {
    provider: "neon",
    url: process.env.DATABASE_URL,
  },
  // Additional configuration...
});
```

### 4. Create API Routes
```javascript
// pages/api/auth/[...auth].ts
import { auth } from "../../../lib/auth";

export default auth;
```

### 5. Add Middleware for Access Control
```javascript
// middleware.ts
import { withAuth } from "next-auth/middleware";

export default withAuth({
  callbacks: {
    authorized: ({ token, req }) => {
      // Check onboarding completion
      return token?.onboardingComplete || req.nextUrl.pathname === "/onboarding";
    }
  }
});
```