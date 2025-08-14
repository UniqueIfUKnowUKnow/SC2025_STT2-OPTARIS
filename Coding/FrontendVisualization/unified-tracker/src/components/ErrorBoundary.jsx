import React from 'react';

// ErrorBoundary Component - This catches any errors that occur during rendering
// If something goes wrong, it shows a friendly error message instead of crashing the app
class ErrorBoundary extends React.Component {
  constructor(props) {
    super(props);
    // Track whether an error has occurred
    this.state = { hasError: false, error: null, errorInfo: null };
  }

  // This method is called when an error occurs during rendering
  static getDerivedStateFromError(error) {
    // Update state to show the error UI
    return { hasError: true };
  }

  // This method is called after an error occurs
  componentDidCatch(error, errorInfo) {
    // Log the error details for debugging
    console.error('Error caught by boundary:', error, errorInfo);
    
    // Store the error information in state
    this.setState({
      error: error,
      errorInfo: errorInfo
    });
  }

  // Function to reset the error state and try again
  handleRetry = () => {
    this.setState({ hasError: false, error: null, errorInfo: null });
  };

  // Function to go back to the start screen
  handleGoHome = () => {
    this.setState({ hasError: false, error: null, errorInfo: null });
    if (this.props.onGoHome) {
      this.props.onGoHome();
    }
  };

  render() {
    // If an error occurred, show the error UI
    if (this.state.hasError) {
      return (
        <div style={{
          position: 'fixed',
          top: 0,
          left: 0,
          width: '100vw',
          height: '100vh',
          background: '#000000',
          color: '#ffffff',
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          padding: '40px',
          textAlign: 'center',
          zIndex: 9999
        }}>
          {/* Error icon */}
          <div style={{
            fontSize: '4rem',
            color: '#ff4444',
            marginBottom: '20px'
          }}>
            ⚠️
          </div>
          
          {/* Error title */}
          <h1 style={{
            fontSize: '2rem',
            color: '#ff4444',
            marginBottom: '20px'
          }}>
            Something went wrong
          </h1>
          
          {/* Error description */}
          <p style={{
            fontSize: '1.1rem',
            color: '#cccccc',
            marginBottom: '30px',
            maxWidth: '600px',
            lineHeight: '1.6'
          }}>
            The application encountered an error while trying to render the 3D scene. 
            This might be due to missing resources or a temporary issue.
          </p>
          
          {/* Error details (only show in development) */}
          {process.env.NODE_ENV === 'development' && this.state.error && (
            <details style={{
              background: 'rgba(255, 68, 68, 0.1)',
              padding: '20px',
              borderRadius: '8px',
              marginBottom: '30px',
              textAlign: 'left',
              maxWidth: '600px'
            }}>
              <summary style={{ cursor: 'pointer', color: '#ff8888' }}>
                Error Details (Click to expand)
              </summary>
              <pre style={{
                color: '#ffaaaa',
                fontSize: '0.9rem',
                overflow: 'auto',
                marginTop: '10px'
              }}>
                {this.state.error.toString()}
                {this.state.errorInfo && this.state.errorInfo.componentStack}
              </pre>
            </details>
          )}
          
          {/* Action buttons */}
          <div style={{
            display: 'flex',
            gap: '20px',
            flexWrap: 'wrap',
            justifyContent: 'center'
          }}>
            {/* Try again button */}
            <button
              onClick={this.handleRetry}
              style={{
                padding: '15px 30px',
                fontSize: '1.1rem',
                background: 'linear-gradient(45deg, #00ff88, #00cc6a)',
                color: '#000000',
                border: 'none',
                borderRadius: '8px',
                cursor: 'pointer',
                fontWeight: 'bold',
                transition: 'all 0.3s ease'
              }}
              onMouseEnter={(e) => {
                e.target.style.transform = 'translateY(-2px)';
                e.target.style.boxShadow = '0 5px 15px rgba(0, 255, 136, 0.4)';
              }}
              onMouseLeave={(e) => {
                e.target.style.transform = 'translateY(0)';
                e.target.style.boxShadow = 'none';
              }}
            >
              Try Again
            </button>
            
            {/* Go home button */}
            <button
              onClick={this.handleGoHome}
              style={{
                padding: '15px 30px',
                fontSize: '1.1rem',
                background: 'rgba(255, 255, 255, 0.1)',
                color: '#ffffff',
                border: '1px solid rgba(255, 255, 255, 0.3)',
                borderRadius: '8px',
                cursor: 'pointer',
                fontWeight: 'bold',
                transition: 'all 0.3s ease'
              }}
              onMouseEnter={(e) => {
                e.target.style.background = 'rgba(255, 255, 255, 0.2)';
                e.target.style.borderColor = 'rgba(255, 255, 255, 0.5)';
              }}
              onMouseLeave={(e) => {
                e.target.style.background = 'rgba(255, 255, 255, 0.1)';
                e.target.style.borderColor = 'rgba(255, 255, 255, 0.3)';
              }}
            >
              Go to Start Screen
            </button>
          </div>
          
          {/* Additional help text */}
          <p style={{
            fontSize: '0.9rem',
            color: '#888888',
            marginTop: '30px',
            maxWidth: '500px'
          }}>
            If this problem persists, try refreshing the page or check your internet connection.
          </p>
        </div>
      );
    }

    // If no error, render the children normally
    return this.props.children;
  }
}

export default ErrorBoundary;
