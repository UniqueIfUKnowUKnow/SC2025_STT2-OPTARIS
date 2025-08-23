import React, { useState, useEffect, useRef } from 'react';

// LoadingScene Component - This shows a loading animation while the 3D scene is initializing
// This prevents the white screen crash by giving the 3D components time to load properly
const LoadingScene = ({ onComplete }) => {
  // Track loading progress
  const [progress, setProgress] = useState(0);
  const [loadingText, setLoadingText] = useState('Initializing 3D Scene...');
  
  // Use a ref to track if we've already completed to prevent multiple calls
  const hasCompleted = useRef(false);
  
  // Simple loading simulation with timeout
  useEffect(() => {
    console.log('LoadingScene: useEffect started');
    
    // Prevent multiple completions
    if (hasCompleted.current) {
      console.log('LoadingScene: Already completed, returning early');
      return;
    }
    
    // Simple progress simulation
    let currentProgress = 0;
    const progressInterval = setInterval(() => {
      currentProgress += 10;
      setProgress(currentProgress);
      
      if (currentProgress >= 100) {
        clearInterval(progressInterval);
        hasCompleted.current = true;
        setLoadingText('Ready!');
        
        // Call onComplete after a short delay
        setTimeout(() => {
          console.log('LoadingScene: Calling onComplete');
          if (onComplete) {
            onComplete();
          }
        }, 500);
      } else if (currentProgress === 20) {
        setLoadingText('Loading Earth textures...');
      } else if (currentProgress === 40) {
        setLoadingText('Initializing satellite data...');
      } else if (currentProgress === 60) {
        setLoadingText('Setting up LiDAR simulation...');
      } else if (currentProgress === 80) {
        setLoadingText('Preparing 3D environment...');
      }
    }, 200);
    
    // Fallback timeout to prevent getting stuck (5 seconds max)
    const fallbackTimeout = setTimeout(() => {
      console.log('LoadingScene: Fallback timeout triggered');
      if (!hasCompleted.current) {
        clearInterval(progressInterval);
        hasCompleted.current = true;
        setProgress(100);
        setLoadingText('Ready!');
        
        setTimeout(() => {
          if (onComplete) {
            console.log('LoadingScene: Fallback onComplete callback');
            onComplete();
          }
        }, 500);
      }
    }, 5000);
    
    return () => {
      console.log('LoadingScene: Cleanup function called');
      clearInterval(progressInterval);
      clearTimeout(fallbackTimeout);
    };
  }, []); // Empty dependency array to run only once

  return (
    <div style={{
      position: 'fixed',
      top: 0,
      left: 0,
      width: '100vw',
      height: '100vh',
      background: '#000000',
      display: 'flex',
      flexDirection: 'column',
      alignItems: 'center',
      justifyContent: 'center',
      zIndex: 1000
    }}>
      {/* Animated logo/icon */}
      <div style={{
        width: '120px',
        height: '120px',
        marginBottom: '40px',
        position: 'relative'
      }}>
        {/* Outer ring */}
        <div style={{
          position: 'absolute',
          top: 0,
          left: 0,
          width: '100%',
          height: '100%',
          border: '4px solid rgba(0, 255, 136, 0.3)',
          borderRadius: '50%',
          animation: 'pulse 2s infinite'
        }} />
        
        {/* Inner ring */}
        <div style={{
          position: 'absolute',
          top: '20px',
          left: '20px',
          width: '80px',
          height: '80px',
          border: '4px solid rgba(0, 255, 136, 0.6)',
          borderRadius: '50%',
          animation: 'pulse 2s infinite 0.5s'
        }} />
        
        {/* Center icon */}
        <div style={{
          position: 'absolute',
          top: '40px',
          left: '40px',
          width: '40px',
          height: '40px',
          background: 'linear-gradient(45deg, #00ff88, #00cc6a)',
          borderRadius: '50%',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          fontSize: '20px',
          color: '#000000',
          fontWeight: 'bold',
          animation: 'rotate 3s linear infinite'
        }}>
          🛰️
        </div>
      </div>
      
      {/* Loading title */}
      <h1 style={{
        fontSize: '2.5rem',
        color: '#ffffff',
        marginBottom: '20px',
        textAlign: 'center',
        fontWeight: '300'
      }}>
        Drone Tracker Digital Twin
      </h1>
      
      {/* Loading subtitle */}
      <p style={{
        fontSize: '1.2rem',
        color: '#cccccc',
        marginBottom: '40px',
        textAlign: 'center'
      }}>
        Initializing 3D visualization system...
      </p>
      
      {/* Progress bar container */}
      <div style={{
        width: '400px',
        maxWidth: '80vw',
        marginBottom: '20px'
      }}>
        {/* Progress bar background */}
        <div style={{
          width: '100%',
          height: '8px',
          background: 'rgba(255, 255, 255, 0.1)',
          borderRadius: '4px',
          overflow: 'hidden'
        }}>
          {/* Progress bar fill */}
          <div style={{
            width: `${progress}%`,
            height: '100%',
            background: 'linear-gradient(90deg, #00ff88, #00cc6a)',
            borderRadius: '4px',
            transition: 'width 0.3s ease',
            boxShadow: '0 0 10px rgba(0, 255, 136, 0.5)'
          }} />
        </div>
      </div>
      
      {/* Progress percentage */}
      <div style={{
        fontSize: '1.1rem',
        color: '#00ff88',
        marginBottom: '20px',
        fontWeight: 'bold'
      }}>
        {progress}%
      </div>
      
      {/* Loading text */}
      <div style={{
        fontSize: '1rem',
        color: '#888888',
        textAlign: 'center',
        maxWidth: '400px',
        lineHeight: '1.5'
      }}>
        {loadingText}
      </div>
      
      {/* Technical info */}
      <div style={{
        position: 'absolute',
        bottom: '40px',
        left: '50%',
        transform: 'translateX(-50%)',
        fontSize: '0.8rem',
        color: '#666666',
        textAlign: 'center'
      }}>
        <p>Powered by React Three Fiber & Three.js</p>
        <p>Real-time satellite tracking & LiDAR simulation</p>
      </div>
      
      {/* CSS animations */}
      <style>{`
        @keyframes pulse {
          0%, 100% { transform: scale(1); opacity: 0.3; }
          50% { transform: scale(1.1); opacity: 0.6; }
        }
        
        @keyframes rotate {
          from { transform: rotate(0deg); }
          to { transform: rotate(360deg); }
        }
      `}</style>
    </div>
  );
};

export default LoadingScene;
