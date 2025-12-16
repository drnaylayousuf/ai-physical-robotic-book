import React, { useState, useEffect } from 'react';
import { useAuth } from '../contexts/AuthProvider';
import Link from '@docusaurus/Link';
import NavbarItem from '@theme/NavbarItem';

// Simple wrapper that conditionally renders sign in/out links based on auth status
const NavbarItemCustomAuthNavButtons = (props) => {
  const { session, isPending, signOut } = useAuth();
  const [showProfileMenu, setShowProfileMenu] = useState(false);

  // Close profile menu when clicking outside
  useEffect(() => {
    const handleClickOutside = (event) => {
      const target = event.target;
      if (!target.closest('.user-profile-dropdown')) {
        setShowProfileMenu(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  const handleSignOut = async () => {
    try {
      await signOut();
      setShowProfileMenu(false);
    } catch (error) {
      console.error('Sign out error:', error);
    }
  };

  if (isPending) {
    // Don't show anything while loading
    return <NavbarItem {...props} />;
  }

  if (session) {
    // Show user profile dropdown when authenticated
    return (
      <div className="navbar__item user-profile-dropdown dropdown dropdown--right">
        <button
          className="navbar__clean-btn dropdown__toggle navbar__item navbar__link"
          onClick={() => setShowProfileMenu(!showProfileMenu)}
          aria-label="User menu"
          aria-haspopup="true"
          style={{ display: 'flex', alignItems: 'center' }}
        >
          <span>{session.user?.name || session.user?.email?.split('@')[0]}</span>
          <span style={{ marginLeft: '0.5rem' }}>▼</span>
        </button>
        {showProfileMenu && (
          <ul className="dropdown__menu">
            <li>
              <Link to="/" className="dropdown__link">Home</Link>
            </li>
            <li>
              <Link to="/profile" className="dropdown__link">Profile</Link>
            </li>
            <li>
              <button
                className="dropdown__link"
                onClick={handleSignOut}
                style={{
                  width: '100%',
                  textAlign: 'left',
                  border: 'none',
                  background: 'none',
                  cursor: 'pointer'
                }}
              >
                Sign Out
              </button>
            </li>
          </ul>
        )}
      </div>
    );
  }

  // Show sign in/up buttons when not authenticated
  return (
    <div className="navbar__item auth-nav-container">
      <Link
        to="/signin"
        className="navbar__item navbar__link"
        style={{ marginRight: '1rem' }}
      >
        Sign In
      </Link>
      <Link
        to="/signup"
        className="navbar__item navbar__link"
      >
        Sign Up
      </Link>
    </div>
  );
};

export default NavbarItemCustomAuthNavButtons;