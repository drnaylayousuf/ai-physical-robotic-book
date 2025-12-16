import React from 'react';
import { useAuth } from '../../contexts/AuthProvider';

// Higher-order component for protecting routes that require authentication
const withAuthProtection = <P extends object>(
  WrappedComponent: React.ComponentType<P>,
  options: {
    redirectIfNotAuthenticated?: string;
    redirectIfOnboardingNotComplete?: string;
  } = {}
) => {
  const { redirectIfNotAuthenticated = '/signin', redirectIfOnboardingNotComplete = '/onboarding' } = options;

  const AuthProtectedComponent: React.FC<P> = (props) => {
    const { session, isPending } = useAuth();

    // In Docusaurus, we'll need to handle redirects differently
    // This is a simplified version - in practice you'd use Docusaurus' navigation methods
    React.useEffect(() => {
      if (!isPending && !session) {
        // Redirect to sign in if not authenticated
        window.location.href = redirectIfNotAuthenticated;
      } else if (!isPending && session && !session.user?.onboardingComplete) {
        // Redirect to onboarding if not completed
        window.location.href = redirectIfOnboardingNotComplete;
      }
    }, [session, isPending]);

    // Show loading state while checking authentication
    if (isPending) {
      return <div>Loading...</div>;
    }

    // If not authenticated or onboarding not complete, don't render the component
    if (!session) {
      return <div>Redirecting to login...</div>;
    }

    // If onboarding is not complete, don't render the component
    if (!session.user?.onboardingComplete) {
      return <div>Redirecting to onboarding...</div>;
    }

    // Render the wrapped component if authenticated and onboarding complete
    return <WrappedComponent {...props as P} />;
  };

  return AuthProtectedComponent;
};

export default withAuthProtection;