import { createAuthClient } from "better-auth/client";

// Create the client instance for Better Auth
const authClient = createAuthClient({
  fetchOptions: {
    baseUrl: typeof window !== 'undefined' ? window.location.origin : process.env.BETTER_AUTH_URL || "http://localhost:3000",
  },
  plugins: [], // Add any client-side plugins if needed
});

// Export the individual methods
export const { signIn, signUp, signOut } = authClient;

// Export the useAuth hook separately
export const useAuth = authClient.useAuth;

// Export the full client if needed
export { authClient };