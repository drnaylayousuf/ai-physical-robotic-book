import React from 'react';
import Head from '@docusaurus/Head';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthProvider';
import './../css/auth.css';

export default function DashboardPage() {
  const { session, isPending } = useAuth();

  if (isPending) {
    return (
      <Layout title="Loading" description="Loading dashboard...">
        <Head>
          <title>Loading - Humanoid Robotics Book</title>
        </Head>
        <div className="auth-container">
          <div className="auth-card">
            <div className="auth-header">
              <h1>Loading Dashboard...</h1>
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
    <Layout title="Dashboard" description="Your personalized dashboard">
      <Head>
        <title>Dashboard - Humanoid Robotics Book</title>
      </Head>
      <div className="auth-container">
        <div className="auth-card">
          <div className="auth-header">
            <h1>Welcome, {session.user?.name || session.user?.email?.split('@')[0]}!</h1>
            <p>Your personalized robotics learning dashboard</p>
          </div>

          <div style={{ textAlign: 'center', padding: '2rem 0' }}>
            <h2>Learning Path</h2>
            <p>Based on your profile, we recommend starting with:</p>
            <ul style={{ textAlign: 'left', listStyle: 'none', padding: 0 }}>
              <li style={{ padding: '0.5rem 0', borderBottom: '1px solid #eee' }}>
                <a href="/docs/intro">Introduction to Humanoid Robotics</a>
              </li>
              <li style={{ padding: '0.5rem 0', borderBottom: '1px solid #eee' }}>
                <a href="/docs/getting-started">Getting Started with Physical AI</a>
              </li>
              <li style={{ padding: '0.5rem 0', borderBottom: '1px solid #eee' }}>
                <a href="/docs/basics">Robotics Basics</a>
              </li>
            </ul>
          </div>
        </div>
      </div>
    </Layout>
  );
}