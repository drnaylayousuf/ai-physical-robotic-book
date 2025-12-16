import React, { useState } from 'react';
import Head from '@docusaurus/Head';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthProvider';
import './../css/auth.css';

export default function SigninPage() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const { signIn } = useAuth();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const response = await signIn({
        email,
        password,
      });

      // Redirect to the book after successful sign in
      if (typeof window !== 'undefined') {
        window.location.href = '/'; // Redirect to main book page
      }
    } catch (err: any) {
      setError(err.message || 'An error occurred during sign in');
      console.error('Signin error:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Sign In" description="Sign in to your account">
      <Head>
        <title>Sign In - Humanoid Robotics Book</title>
      </Head>
      <div className="auth-container">
        <div className="auth-card">
          <div className="auth-header">
            <h1>Welcome Back</h1>
            <p>Sign in to continue your robotics journey</p>
          </div>

          {error && <div className="error-message">{error}</div>}

          <form onSubmit={handleSubmit} className="auth-form">
            <div className="form-group">
              <label htmlFor="email">Email Address</label>
              <input
                type="email"
                id="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                placeholder="Enter your email"
                required
              />
            </div>

            <div className="form-group">
              <label htmlFor="password">Password</label>
              <input
                type="password"
                id="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                placeholder="Enter your password"
                required
              />
            </div>

            <button type="submit" className="auth-button" disabled={loading}>
              {loading ? (
                <>
                  <span className="loading-spinner"></span>
                  <span>Signing In...</span>
                </>
              ) : (
                'Sign In'
              )}
            </button>
          </form>

          <div className="auth-footer">
            <p>
              Don't have an account?{' '}
              <a href="/signup">Sign up</a>
            </p>
            <p>
              <a href="#password-reset">Forgot password?</a>
            </p>
          </div>
        </div>
      </div>
    </Layout>
  );
}