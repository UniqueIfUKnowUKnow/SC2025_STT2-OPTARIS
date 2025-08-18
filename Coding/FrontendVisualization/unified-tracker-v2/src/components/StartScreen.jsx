import React from 'react';
import './StartScreen.css';

// StartScreen Component
// This is the first thing users see when they open the application
// It provides a clean, professional entry point before showing the complex 3D tracking interface
const StartScreen = ({ onStart }) => {
  // This function gets the current date and time to display on the start screen
  // It updates every second to show real-time information
  const [currentDateTime, setCurrentDateTime] = React.useState(new Date());

  // This effect runs every second to update the date and time display
  // It's like having a digital clock that's always current
  React.useEffect(() => {
    // Create a timer that updates every 1000 milliseconds (1 second)
    const timer = setInterval(() => {
      setCurrentDateTime(new Date());  // Get the current date and time
    }, 1000);

    // Cleanup function - this runs when the component is removed from the screen
    // It stops the timer so we don't waste computer resources
    return () => clearInterval(timer);
  }, []); // Empty array means this effect runs once when the component is created

  // Function to format the date in a readable way (e.g., "January 15, 2024")
  const formatDate = (date) => {
    return date.toLocaleDateString('en-US', {
      weekday: 'long',      // Full day name (e.g., "Monday")
      year: 'numeric',      // Full year (e.g., "2024")
      month: 'long',        // Full month name (e.g., "January")
      day: 'numeric'        // Day of the month (e.g., "15")
    });
  };

  // Function to format the time in a readable way (e.g., "2:30:45 PM")
  const formatTime = (date) => {
    return date.toLocaleTimeString('en-US', {
      hour: 'numeric',      // Hour (e.g., "2")
      minute: '2-digit',    // Minute with leading zero (e.g., "30")
      second: '2-digit',    // Second with leading zero (e.g., "45")
      hour12: true          // Use 12-hour format with AM/PM
    });
  };

  // This is what gets displayed on the screen
  return (
    <div className="start-screen">
      {/* Background container that covers the entire screen */}
      <div className="start-background">
        
        {/* Main content area centered on the screen */}
        <div className="start-content">
          
          {/* Application title - this is the main heading users see */}
          <h1 className="app-title">
            Drone Tracker Digital Twin
          </h1>
          
          {/* Subtitle that explains what the application does */}
          <p className="app-subtitle">
            Advanced satellite tracking and LiDAR monitoring system
          </p>
          
          {/* Description of what the system can do */}
          <div className="app-description">
            <p>
              This system provides real-time tracking of satellites in Earth orbit 
              and local LiDAR-based object detection and monitoring capabilities.
            </p>
            <p>
              Features include 3D visualisation of Earth and satellites, 
              ground station monitoring, and precise orbital calculations.
            </p>
          </div>
          
          {/* Start button - clicking this launches the main application */}
          <button 
            className="start-button" 
            onClick={onStart}
            // This makes the button accessible to screen readers and keyboard navigation
            aria-label="Start the Drone Tracker application"
          >
            Start Application
          </button>
          
          {/* Technical information section */}
          <div className="tech-info">
            <p>System Status: Ready</p>
            <p>Version: 2.0.0</p>
            <p>Last Updated: August 2025</p>
          </div>
        </div>
        
        {/* Date and time display at the bottom of the screen */}
        {/* This shows the current date and time, updating every second */}
        <div className="datetime-display">
          <div className="date-text">
            {formatDate(currentDateTime)}
          </div>
          <div className="time-text">
            {formatTime(currentDateTime)}
          </div>
        </div>
        
        {/* Decorative elements to make the screen more visually appealing */}
        <div className="decorative-elements">
          {/* Animated dots that represent the system "thinking" or being ready */}
          <div className="status-dots">
            <div className="dot active"></div>
            <div className="dot active"></div>
            <div className="dot active"></div>
          </div>
        </div>
      </div>
    </div>
  );
};

// Export the StartScreen component so it can be used by the main App component
export default StartScreen;
