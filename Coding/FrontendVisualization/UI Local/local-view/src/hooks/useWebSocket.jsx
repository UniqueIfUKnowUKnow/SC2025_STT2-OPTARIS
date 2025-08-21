import { useState, useCallback, useEffect } from 'react';

const useWebSocket = () => {
  const [points, setPoints] = useState([]);
  const [isConnected, setIsConnected] = useState(false);
  const [error, setError] = useState(null);
  const [ws, setWs] = useState(null);

  const disconnect = useCallback(() => {
    if (ws) {
      ws.close();
      setWs(null);
      setIsConnected(false);
    }
  }, [ws]);

  const connect = useCallback((url) => {
    if (!url) {
      setError('Please provide a valid WebSocket URL');
      return;
    }

    // Disconnect existing connection if any
    disconnect();

    try {
      const newWs = new WebSocket(url);

      newWs.onopen = () => {
        console.log('WebSocket Connected');
        setIsConnected(true);
        setError(null);
        setWs(newWs);
      };

      newWs.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (Array.isArray(data)) {
            const validPoints = data.filter(point => 
              typeof point === 'object' &&
              'x' in point &&
              'y' in point &&
              'z' in point &&
              'timestamp' in point &&
              !isNaN(point.x) &&
              !isNaN(point.y) &&
              !isNaN(point.z) &&
              !isNaN(point.timestamp)
            );
            
            setPoints(prevPoints => [...prevPoints, ...validPoints]);
          }
        } catch (err) {
          console.error('Error parsing WebSocket data:', err);
        }
      };

      newWs.onerror = (error) => {
        console.error('WebSocket Error:', error);
        setError('WebSocket connection error');
        setIsConnected(false);
        setWs(null);
      };

      newWs.onclose = () => {
        console.log('WebSocket Disconnected');
        setIsConnected(false);
        setWs(null);
      };
    } catch (err) {
      console.error('Error creating WebSocket:', err);
      setError(err.message);
      setIsConnected(false);
      setWs(null);
    }
  }, [disconnect]);

  const clearPoints = useCallback(() => {
    setPoints([]);
  }, []);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (ws) {
        ws.close();
      }
    };
  }, [ws]);

  return {
    points,
    isConnected,
    error,
    connect,
    disconnect,
    clearPoints
  };
};

export default useWebSocket;
