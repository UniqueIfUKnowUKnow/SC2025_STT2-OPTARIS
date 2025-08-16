import React, { useState, useEffect, useRef } from 'react';

// You will need to replace this with the actual IP address of your Raspberry Pi.
// Example: 'ws://192.168.1.100:8765'
const PI_IP_ADDRESS = '10.111.244.104';
const PI_PORT = 8765;

// Main App component
const App = () => {
  const [data, setData] = useState(null);
  const [connectionStatus, setConnectionStatus] = useState('Connecting...');
  const ws = useRef(null);

  // This useEffect hook manages the WebSocket connection lifecycle
  useEffect(() => { 
    // Only connect if a valid IP address is provided
    if (PI_IP_ADDRESS === '10.111.244.104') {
      setConnectionStatus("Please set your Raspberry Pi's IP address.");
      return;
    }

    // Create a new WebSocket connection
    ws.current = new WebSocket(`ws://${PI_IP_ADDRESS}:${PI_PORT}`);

    // Handler for when the connection is successfully opened
    ws.current.onopen = () => {
      console.log("WebSocket connected to Raspberry Pi.");
      setConnectionStatus("Connected!");
    };

    // Handler for incoming messages from the server
    ws.current.onmessage = (event) => {
      try {
        // Parse the incoming JSON data
        const receivedData = JSON.parse(event.data);
        console.log("Received data:", receivedData);
        // Update the component's state with the new data
        setData(receivedData);
      } catch (error) {
        console.error("Failed to parse JSON:", error);
      }
    };

    // Handler for when the connection is closed
    ws.current.onclose = () => {
      console.log("WebSocket disconnected.");
      setConnectionStatus("Disconnected. Retrying in 3 seconds...");
      // Attempt to reconnect after a short delay
      setTimeout(() => {
        if (ws.current) {
          // Re-establish the connection by creating a new WebSocket instance
          ws.current = new WebSocket(`ws://${PI_IP_ADDRESS}:${PI_PORT}`);
        }
      }, 3000);
    };

    // Handler for WebSocket errors
    ws.current.onerror = (error) => {
      console.error("WebSocket error:", error);
      setConnectionStatus("Error. Check console for details.");
    };

    // Cleanup function: this runs when the component unmounts
    return () => {
      if (ws.current) {
        ws.current.close();
      }
    };
  }, []); // Empty dependency array ensures this effect runs only once on mount

  // Function to send data back to the Raspberry Pi
  const sendData = () => {
    if (ws.current && ws.current.readyState === WebSocket.OPEN) {
      // --- Easily change the data you want to send here ---
      const message = {
        command: "toggle_led",
        value: true,
        from: "React App"
      };
      ws.current.send(JSON.stringify(message));
      console.log("Sent message to Pi:", message);
    } else {
      console.warn("WebSocket is not open. Cannot send message.");
    }
  };

  return (
    <div className="flex flex-col items-center justify-center min-h-screen bg-gray-100 p-4 font-sans text-gray-800">
      <div className="bg-white shadow-xl rounded-2xl p-8 max-w-lg w-full text-center">
        <h1 className="text-4xl font-extrabold text-blue-600 mb-2">Real-Time Data Stream</h1>
        <p className="text-gray-600 mb-6">From Raspberry Pi to React</p>
        
        <div className="space-y-4">
          <div className="bg-gray-50 rounded-lg p-4 shadow-inner">
            <p className="text-lg font-semibold text-gray-700">Connection Status:</p>
            <span className={`text-xl font-bold ${connectionStatus === 'Connected!' ? 'text-green-500' : 'text-red-500'}`}>
              {connectionStatus}
            </span>
          </div>

          <div className="bg-gray-50 rounded-lg p-4 shadow-inner">
            <p className="text-lg font-semibold text-gray-700">Latest Data from Pi:</p>
            {data ? (
              <div className="mt-2 text-left">
                <p className="text-sm font-mono text-gray-900">
                  <span className="font-bold">ID:</span> {data.id}
                </p>
                <p className="text-sm font-mono text-gray-900">
                  <span className="font-bold">Value:</span> {data.value.toFixed(2)}
                </p>
                <p className="text-sm font-mono text-gray-900">
                  <span className="font-bold">Timestamp:</span> {new Date(data.timestamp).toLocaleTimeString()}
                </p>
              </div>
            ) : (
              <p className="text-sm text-gray-500 mt-2">Waiting for data...</p>
            )}
          </div>
        </div>

        <button 
          onClick={sendData}
          className="mt-8 px-6 py-3 bg-blue-600 text-white font-bold rounded-full shadow-lg hover:bg-blue-700 transition duration-300 transform hover:scale-105 active:scale-95 disabled:bg-gray-400 disabled:shadow-none"
          disabled={connectionStatus !== 'Connected!'}
        >
          Send Message to Pi
        </button>
      </div>
    </div>
  );
};

export default App;
