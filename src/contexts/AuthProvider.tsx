import React, { ReactNode, createContext, useContext, useEffect, useState } from 'react';
import { CustomAuthAPI } from '../lib/custom-auth-client.tsx';

// Define the auth context type
interface AuthContextType {
  session: any;
  isPending: boolean;
  error: Error | null;
  signIn: (credentials: { email: string; password: string }) => Promise<any>;
  signUp: (userData: { email: string; password: string; name?: string }) => Promise<any>;
  signOut: () => Promise<any>;
  getSession: () => Promise<any>;
  updateOnboarding: (data: {
    technicalBackground?: string;
    experienceLevel?: string;
    interests?: string;
    goals?: string;
  }) => Promise<any>;
}

// Create the context with a default undefined value
const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [session, setSession] = useState<any>(null);
  const [isPending, setIsPending] = useState<boolean>(true);
  const [error, setError] = useState<Error | null>(null);

  useEffect(() => {
    // Initialize auth state when component mounts
    const initializeAuth = async () => {
      setIsPending(true);
      setError(null);

      try {
        const authAPI = new CustomAuthAPI();
        // Add a timeout to prevent indefinite loading
        const timeoutPromise = new Promise((_, reject) => {
          setTimeout(() => reject(new Error('Session initialization timeout')), 10000); // 10 second timeout
        });

        // Race the API call with a timeout
        const sessionData = await Promise.race([
          authAPI.getSession(),
          timeoutPromise
        ]) as any;

        setSession(sessionData);
      } catch (err: any) {
        console.error('Auth initialization error:', err);
        // Don't set an error if it's just a timeout, allow for slower loading
        if (err.message && err.message.includes('timeout')) {
          console.warn('Session loading taking longer than expected, continuing with default state');
          setSession({ onboardingComplete: false });
        } else {
          setError(err as Error);
          setSession({ onboardingComplete: false });
        }
      } finally {
        // Always set isPending to false after initialization attempt
        setIsPending(false);
      }
    };

    initializeAuth();
  }, []);

  const signIn = async (credentials: { email: string; password: string }) => {
    setIsPending(true);
    setError(null);

    try {
      const authAPI = new CustomAuthAPI();
      const result = await authAPI.signIn(credentials);
      setSession(result);
      return result;
    } catch (err) {
      setError(err as Error);
      throw err;
    } finally {
      setIsPending(false);
    }
  };

  const signUp = async (userData: { email: string; password: string; name?: string }) => {
    setIsPending(true);
    setError(null);

    try {
      const authAPI = new CustomAuthAPI();
      const result = await authAPI.signUp(userData);
      setSession(result);
      return result;
    } catch (err) {
      setError(err as Error);
      throw err;
    } finally {
      setIsPending(false);
    }
  };

  const signOut = async () => {
    setIsPending(true);
    setError(null);

    try {
      const authAPI = new CustomAuthAPI();
      const result = await authAPI.signOut();
      setSession({ onboardingComplete: false });
      return result;
    } catch (err) {
      setError(err as Error);
      throw err;
    } finally {
      setIsPending(false);
    }
  };

  const getSession = async () => {
    try {
      const authAPI = new CustomAuthAPI();
      const result = await authAPI.getSession();
      setSession(result);
      return result;
    } catch (err) {
      setError(err as Error);
      throw err;
    }
  };

  const updateOnboarding = async (data: {
    technicalBackground?: string;
    experienceLevel?: string;
    interests?: string;
    goals?: string;
  }) => {
    try {
      const authAPI = new CustomAuthAPI();
      const result = await authAPI.updateOnboarding(data);
      // Update session if needed
      if (result.profile) {
        setSession(prev => ({ ...prev, profile: result.profile, onboardingComplete: result.onboardingComplete }));
      }
      return result;
    } catch (err) {
      setError(err as Error);
      throw err;
    }
  };

  const contextValue = {
    session,
    isPending,
    error,
    signIn,
    signUp,
    signOut,
    getSession,
    updateOnboarding
  };

  return (
    <AuthContext.Provider value={contextValue}>
      {children}
    </AuthContext.Provider>
  );
};

export default AuthProvider;

// Export the context for use in components
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    console.error('useAuth must be used within an AuthProvider');
    // Return a default context to prevent crashes
    return {
      session: null,
      isPending: true,
      error: null,
      signIn: () => {},
      signUp: () => {},
      signOut: () => {},
      getSession: () => {},
      updateOnboarding: () => {}
    };
  }
  return context;
};