// src/main.jsx
// This is the main entry point for the React application
// It sets up the root element and renders the App component

// Import StrictMode from React for additional development-time checks
// StrictMode helps identify potential problems in the application
import { StrictMode } from 'react'

// Import createRoot from React DOM for creating the root element
// This is the modern way to render React applications (React 18+)
import { createRoot } from 'react-dom/client'

// Import global CSS styles that apply to the entire application
import './index.css'

// Import the main App component that contains our application logic
import App from './App.jsx'

// Create the root element and render the application
// document.getElementById('root') finds the HTML element with id="root"
// This element should exist in the public/index.html file
createRoot(document.getElementById('root')).render(
  // Wrap the App component in StrictMode for development benefits
  <StrictMode>
    {/* Render the main App component */}
    <App />
  </StrictMode>,
)
