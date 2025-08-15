import { useState, useEffect, useCallback } from 'react';
import { useStore } from '../useDroneStore';

const WS_URL = 'ws://localhost:8765';

export const useWebSocket = () => {
  const [ws, setWs] = useState(null);
  const [isConnected, setIsConnected] = useState(false);
  const updateCoordinates = useStore((state) => state.updateCoordinates);
  const updatePrediction = useStore((state) => state.updatePrediction);

  const connect = useCallback(() => {
    const websocket = new WebSocket(WS_URL);

    websocket.onopen = () => {
      console.log('Connected to WebSocket');
      setIsConnected(true);
    };

    websocket.onclose = () => {
      console.log('Disconnected from WebSocket');
      setIsConnected(false);
      // Try to reconnect after 2 seconds
      setTimeout(connect, 2000);
    };

    websocket.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    websocket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        
        switch (data.type) {
          case 'coordinates':
            updateCoordinates(data.data);
            break;
          case 'prediction':
            updatePrediction(data.data);
            break;
          default:
            console.log('Unknown message type:', data.type);
        }
      } catch (error) {
        console.error('Error parsing WebSocket message:', error);
      }
    };

    setWs(websocket);
  }, [updateCoordinates, updatePrediction]);

  useEffect(() => {
    connect();
    return () => {
      if (ws) {
        ws.close();
      }
    };
  }, [connect]);

  const sendMessage = useCallback((message) => {
    if (ws && isConnected) {
      ws.send(JSON.stringify(message));
    }
  }, [ws, isConnected]);

  return {
    isConnected,
    sendMessage
  };
};
