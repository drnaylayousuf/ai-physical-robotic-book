// Custom auth client that works with FastAPI backend
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

class CustomAuthClient {
  private baseUrl: string;
  private token: string | null = null;

  constructor() {
    this.baseUrl = typeof window !== 'undefined'
      ? `${window.location.protocol}//${window.location.host}`
      : process.env.BACKEND_URL || 'http://localhost:8000';

    // Try to get token from localStorage on initialization
    if (typeof window !== 'undefined') {
      this.token = localStorage.getItem('auth_token');
    }
  }

  private async request(endpoint: string, options: RequestInit = {}) {
    const url = `${this.baseUrl}${endpoint}`;

    const headers = {
      'Content-Type': 'application/json',
      ...options.headers,
    };

    if (this.token) {
      (headers as Record<string, string>)['Authorization'] = `Bearer ${this.token}`;
    }

    const response = await fetch(url, {
      ...options,
      headers,
    });

    if (!response.ok) {
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
        this.token = response.session.token;
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
        this.token = response.session.token;
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
      const result = await this.request('/sign-out', {
        method: 'POST',
      });

      this.token = null;
      if (typeof window !== 'undefined') {
        localStorage.removeItem('auth_token');
      }

      return result;
    } catch (error) {
      console.error('Sign out error:', error);
      // Even if the server request fails, clear the local token
      this.token = null;
      if (typeof window !== 'undefined') {
        localStorage.removeItem('auth_token');
      }
      return { success: true };
    }
  }

  async getSession(): Promise<AuthResponse> {
    if (!this.token) {
      return { onboardingComplete: false };
    }

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

// Create a singleton instance
const customAuthClient = new CustomAuthClient();

// Export the individual methods
export const signIn = customAuthClient.signIn.bind(customAuthClient);
export const signUp = customAuthClient.signUp.bind(customAuthClient);
export const signOut = customAuthClient.signOut.bind(customAuthClient);


// Export the full client if needed
export { customAuthClient };