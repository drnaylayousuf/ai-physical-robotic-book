import { useState, useEffect, createContext, useContext, ReactNode } from 'react';

interface User {
  id: string;
  email: string;
  name?: string;
}

interface Session {
  user: User;
  token: string;
}

interface UserProfile {
  userId: string;
  technicalBackground?: string;
  experienceLevel?: string;
  interests?: string;
  goals?: string;
  onboardingComplete: boolean;
}

interface AuthResponse {
  user?: User;
  session?: Session;
  profile?: UserProfile;
  onboardingComplete: boolean;
}

interface AuthContextType {
  data: AuthResponse | null;
  isPending: boolean;
  error: Error | null;
  signIn: (credentials: { email: string; password: string }) => Promise<AuthResponse>;
  signUp: (userData: { email: string; password: string; name?: string }) => Promise<AuthResponse>;
  signOut: () => Promise<{ success: boolean }>;
  getSession: () => Promise<AuthResponse>;
  updateOnboarding: (data: {
    userId: string;
    pythonProficiency: string;
    operatingSystem: string;
    preferredEnvironment: string;
  }) => Promise<{ profile?: UserProfile; onboardingComplete: boolean }>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export class CustomAuthAPI {
  private baseUrl: string;

  constructor() {
    this.baseUrl = typeof window !== 'undefined'
      ? `${window.location.protocol}//${window.location.host}`
      : process.env.BACKEND_URL || 'http://localhost:8000';
  }

  private async request(endpoint: string, options: RequestInit = {}) {
    const url = `${this.baseUrl}${endpoint}`;

    const headers = {
      'Content-Type': 'application/json',
      ...options.headers,
    };

    // Get token from localStorage
    const token = typeof window !== 'undefined' ? localStorage.getItem('auth_token') : null;
    if (token) {
      (headers as Record<string, string>)['Authorization'] = `Bearer ${token}`;
    }

    const response = await fetch(url, {
      ...options,
      headers,
    });

    if (!response.ok) {
      // If it's a 401 error, clear the token
      if (response.status === 401) {
        if (typeof window !== 'undefined') {
          localStorage.removeItem('auth_token');
        }
      }
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  }

  async signIn(credentials: { email: string; password: string }): Promise<AuthResponse> {
    try {
      const response = await this.request('/sign-in/email', {
        method: 'POST',
        body: JSON.stringify(credentials),
      });

      if (response.session?.token) {
        if (typeof window !== 'undefined') {
          localStorage.setItem('auth_token', response.session.token);
        }
      }

      return response;
    } catch (error) {
      console.error('Sign in error:', error);
      throw error;
    }
  }

  async signUp(userData: { email: string; password: string; name?: string }): Promise<AuthResponse> {
    try {
      const response = await this.request('/sign-up/email', {
        method: 'POST',
        body: JSON.stringify(userData),
      });

      if (response.session?.token) {
        if (typeof window !== 'undefined') {
          localStorage.setItem('auth_token', response.session.token);
        }
      }

      return response;
    } catch (error) {
      console.error('Sign up error:', error);
      throw error;
    }
  }

  async signOut(): Promise<{ success: boolean }> {
    try {
      // Only try to call the backend if we have a token
      let result = { success: true };
      const token = typeof window !== 'undefined' ? localStorage.getItem('auth_token') : null;

      if (token) {
        result = await this.request('/sign-out', {
          method: 'POST',
        });
      }

      // Clear the local token regardless of server response
      if (typeof window !== 'undefined') {
        localStorage.removeItem('auth_token');
      }

      return result;
    } catch (error) {
      console.error('Sign out error:', error);
      // Even if the server request fails, clear the local token
      if (typeof window !== 'undefined') {
        localStorage.removeItem('auth_token');
      }
      return { success: true };
    }
  }

  async getSession(): Promise<AuthResponse> {
    try {
      const response = await this.request('/session');
      return response;
    } catch (error) {
      console.error('Get session error:', error);
      // Return a response indicating no valid session
      return { onboardingComplete: false };
    }
  }

  async updateOnboarding(data: {
    technicalBackground?: string;
    experienceLevel?: string;
    interests?: string;
    goals?: string;
  }): Promise<{ profile?: UserProfile; onboardingComplete: boolean }> {
    try {
      const response = await this.request('/onboarding', {
        method: 'POST',
        body: JSON.stringify(data),
      });

      return response;
    } catch (error) {
      console.error('Update onboarding error:', error);
      throw error;
    }
  }
}

const authAPI = new CustomAuthAPI();

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider = ({ children }: AuthProviderProps) => {
  const [data, setData] = useState<AuthResponse | null>(null);
  const [isPending, setIsPending] = useState<boolean>(true);
  const [error, setError] = useState<Error | null>(null);

  useEffect(() => {
    // Initialize auth state when component mounts
    const initializeAuth = async () => {
      setIsPending(true);
      setError(null);

      try {
        const session = await authAPI.getSession();
        setData(session);
      } catch (err) {
        setError(err as Error);
        setData({ onboardingComplete: false });
      } finally {
        setIsPending(false);
      }
    };

    initializeAuth();
  }, []);

  const signIn = async (credentials: { email: string; password: string }) => {
    setIsPending(true);
    setError(null);

    try {
      const result = await authAPI.signIn(credentials);
      setData(result);
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
      const result = await authAPI.signUp(userData);
      setData(result);
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
      const result = await authAPI.signOut();
      setData({ onboardingComplete: false });
      return result;
    } catch (err) {
      setError(err as Error);
      throw err;
    } finally {
      setIsPending(false);
    }
  };

  const value: AuthContextType = {
    data,
    isPending,
    error,
    signIn,
    signUp,
    signOut,
    getSession: authAPI.getSession.bind(authAPI),
    updateOnboarding: authAPI.updateOnboarding.bind(authAPI),
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};