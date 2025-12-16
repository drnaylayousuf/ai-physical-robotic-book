import React, { useState, useEffect } from 'react';
import Head from '@docusaurus/Head';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthProvider';
import './../css/auth.css';

export default function ProfilePage() {
  const { session, isPending } = useAuth();
  const [name, setName] = useState('');
  const [email, setEmail] = useState('');
  const [pythonProficiency, setPythonProficiency] = useState('BEGINNER');
  const [operatingSystem, setOperatingSystem] = useState('WINDOWS');
  const [preferredEnvironment, setPreferredEnvironment] = useState('LOCAL_MACHINE');
  const [loading, setLoading] = useState(true);
  const [updating, setUpdating] = useState(false);
  const [message, setMessage] = useState('');

  useEffect(() => {
    const fetchProfile = async () => {
      if (session && !isPending) {
        setName(session.user?.name || '');
        setEmail(session.user?.email || '');

        try {
          // Fetch user profile data
          // Get the auth token from localStorage (as used by the custom auth client)
          const token = typeof window !== 'undefined' ? localStorage.getItem('auth_token') : null;

          const response = await fetch('/api/auth/profile', {
            method: 'GET',
            headers: {
              'Authorization': token || '',
              'Content-Type': 'application/json',
            },
          });

          if (response.ok) {
            const data = await response.json();
            if (data.profile) {
              setPythonProficiency(data.profile.pythonProficiency || 'BEGINNER');
              setOperatingSystem(data.profile.operatingSystem || 'WINDOWS');
              setPreferredEnvironment(data.profile.preferredEnvironment || 'LOCAL_MACHINE');
            }
          }
        } catch (error) {
          console.error('Error fetching profile:', error);
        } finally {
          setLoading(false);
        }
      }
    };

    fetchProfile();
  }, [session, isPending]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setUpdating(true);
    setMessage('');

    try {
      // Get the auth token from localStorage (as used by the custom auth client)
      const token = typeof window !== 'undefined' ? localStorage.getItem('auth_token') : null;

      // Make API call to update profile
      const response = await fetch('/api/auth/profile', {
        method: 'PUT',
        headers: {
          'Authorization': token || '',
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          name,
          pythonProficiency,
          operatingSystem,
          preferredEnvironment
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.error || 'Failed to update profile');
      }

      setMessage('Profile updated successfully!');
    } catch (error) {
      setMessage('Error updating profile: ' + (error as Error).message);
    } finally {
      setUpdating(false);
    }
  };

  if (isPending || loading) {
    return (
      <Layout title="Loading Profile" description="Loading profile information...">
        <Head>
          <title>Loading Profile - Humanoid Robotics Book</title>
        </Head>
        <div className="auth-container">
          <div className="auth-card">
            <div className="auth-header">
              <h1>Loading Profile...</h1>
            </div>
            <div style={{ textAlign: 'center', padding: '2rem' }}>
              <div className="loading-spinner" style={{ margin: '0 auto' }}></div>
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
    <Layout title="Profile" description="Manage your profile information">
      <Head>
        <title>Profile - Humanoid Robotics Book</title>
      </Head>
      <div className="auth-container">
        <div className="auth-card">
          <div className="auth-header">
            <h1>Your Profile</h1>
            <p>Manage your account and preferences</p>
          </div>

          {message && (
            <div className={`error-message ${message.includes('Error') ? '' : 'success-message'}`} style={{
              backgroundColor: message.includes('Error') ? '#fdf2f2' : '#f0fdf4',
              borderLeft: message.includes('Error') ? '4px solid #e74c3c' : '4px solid #10b981',
              color: message.includes('Error') ? '#dc2626' : '#065f46'
            }}>
              {message}
            </div>
          )}

          <form onSubmit={handleSubmit} className="auth-form">
            <div className="form-group">
              <label htmlFor="name">Full Name</label>
              <input
                type="text"
                id="name"
                value={name}
                onChange={(e) => setName(e.target.value)}
                placeholder="Enter your full name"
                required
              />
            </div>

            <div className="form-group">
              <label htmlFor="email">Email Address</label>
              <input
                type="email"
                id="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                placeholder="Enter your email"
                disabled
                style={{ backgroundColor: '#f1f5f9', cursor: 'not-allowed' }}
              />
            </div>

            <div className="form-group">
              <label htmlFor="pythonProficiency">Python Proficiency</label>
              <select
                id="pythonProficiency"
                value={pythonProficiency}
                onChange={(e) => setPythonProficiency(e.target.value)}
              >
                <option value="BEGINNER">Beginner</option>
                <option value="INTERMEDIATE">Intermediate</option>
                <option value="EXPERT">Expert</option>
              </select>
            </div>

            <div className="form-group">
              <label htmlFor="operatingSystem">Operating System</label>
              <select
                id="operatingSystem"
                value={operatingSystem}
                onChange={(e) => setOperatingSystem(e.target.value)}
              >
                <option value="WINDOWS">Windows</option>
                <option value="MACOS">macOS</option>
                <option value="LINUX">Linux</option>
                <option value="OTHER">Other</option>
              </select>
            </div>

            <div className="form-group">
              <label htmlFor="preferredEnvironment">Preferred Environment</label>
              <select
                id="preferredEnvironment"
                value={preferredEnvironment}
                onChange={(e) => setPreferredEnvironment(e.target.value)}
              >
                <option value="LOCAL_MACHINE">Local Machine</option>
                <option value="CLOUD_ENVIRONMENT">Cloud Environment</option>
              </select>
            </div>

            <button type="submit" className="auth-button" disabled={updating}>
              {updating ? (
                <>
                  <span className="loading-spinner"></span>
                  <span>Updating...</span>
                </>
              ) : (
                'Update Profile'
              )}
            </button>
          </form>
        </div>
      </div>
    </Layout>
  );
}