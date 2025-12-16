import React, { useState, useEffect } from 'react';
import Head from '@docusaurus/Head';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthProvider';
import './../css/auth.css';

export default function OnboardingPage() {
  const { session, isPending } = useAuth();
  const [operatingSystem, setOperatingSystem] = useState('');
  const [preferredEnvironment, setPreferredEnvironment] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');

    // Validate required fields
    if (!operatingSystem || !preferredEnvironment) {
      setError('Please answer all questions');
      return;
    }

    setLoading(true);

    try {
      // Call the onboarding API endpoint with the user's responses
      // Get the user ID from the session
      const userId = session?.user?.id;

      if (!userId) {
        throw new Error('User not authenticated');
      }

      // Make direct API call to the onboarding endpoint
      const response = await fetch('/api/auth/onboarding', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          userId,
          pythonProficiency: "BEGINNER", // default value since we removed this question
          operatingSystem,
          preferredEnvironment
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.error || 'Failed to complete onboarding');
      }

      // Redirect to the book after successful onboarding
      if (typeof window !== 'undefined') {
        window.location.href = '/'; // Redirect to main book page
      }
    } catch (err) {
      setError('An error occurred while saving your profile');
      console.error('Onboarding error:', err);
    } finally {
      setLoading(false);
    }
  };

  if (isPending) {
    // Add a timeout to automatically redirect if session takes too long to load
    React.useEffect(() => {
      const timer = setTimeout(() => {
        if (typeof window !== 'undefined') {
          // Force a refresh to reinitialize auth state
          window.location.reload();
        }
      }, 15000); // 15 seconds

      return () => clearTimeout(timer);
    }, []);

    return (
      <Layout title="Loading" description="Loading onboarding...">
        <Head>
          <title>Onboarding - Humanoid Robotics Book</title>
        </Head>
        <div className="auth-container">
          <div className="auth-card">
            <div className="auth-header">
              <h1>Loading Onboarding...</h1>
              <p>Please wait while we prepare your personalized experience</p>
            </div>
            <div style={{ textAlign: 'center', padding: '2rem' }}>
              <div className="loading-spinner" style={{ margin: '0 auto' }}></div>
              <p style={{ marginTop: '1rem', fontSize: '0.9rem', color: '#666' }}>
                If this takes too long, the page will automatically refresh.
              </p>
              <button
                onClick={() => window.location.reload()}
                style={{
                  marginTop: '1rem',
                  padding: '0.5rem 1rem',
                  backgroundColor: '#3498db',
                  color: 'white',
                  border: 'none',
                  borderRadius: '4px',
                  cursor: 'pointer'
                }}
              >
                Refresh Page
              </button>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  if (!session) {
    // Redirect to sign in if not authenticated
    if (typeof window !== 'undefined') {
      window.location.href = '/signin';
    }
    return <div>Redirecting to sign in...</div>;
  }

  return (
    <Layout title="Onboarding" description="Complete your profile to personalize your learning experience">
      <Head>
        <title>Onboarding - Humanoid Robotics Book</title>
      </Head>
      <div className="auth-container">
        <div className="auth-card">
          <div className="auth-header">
            <h1>Welcome to Humanoid Robotics!</h1>
            <p>Tell us about your background to personalize your learning experience</p>
          </div>

          {error && <div className="error-message">{error}</div>}

          <form onSubmit={handleSubmit} className="auth-form">
            <div className="form-group">
              <label htmlFor="operatingSystem">Which operating system do you use?</label>
              <select
                id="operatingSystem"
                value={operatingSystem}
                onChange={(e) => setOperatingSystem(e.target.value)}
                required
              >
                <option value="">Select your OS</option>
                <option value="WINDOWS">Windows</option>
                <option value="MACOS">macOS</option>
                <option value="LINUX">Linux</option>
                <option value="OTHER">Other</option>
              </select>
            </div>

            <div className="form-group">
              <label htmlFor="preferredEnvironment">Which hardware environment do you use?</label>
              <select
                id="preferredEnvironment"
                value={preferredEnvironment}
                onChange={(e) => setPreferredEnvironment(e.target.value)}
                required
              >
                <option value="">Select your preference</option>
                <option value="LOCAL_MACHINE">Local Machine</option>
                <option value="CLOUD_ENVIRONMENT">Cloud Environment</option>
              </select>
            </div>

            <button type="submit" className="auth-button" disabled={loading}>
              {loading ? (
                <>
                  <span className="loading-spinner"></span>
                  <span>Completing Profile...</span>
                </>
              ) : (
                'Complete Onboarding'
              )}
            </button>
          </form>
        </div>
      </div>
    </Layout>
  );
}