import { useState, useCallback, useEffect } from 'react';

const useWebSocket = () => {
  const [points, setPoints] = useState([]);
  const [isConnected, setIsConnected] = useState(false);
  const [error, setError] = useState(null);
  const [ws, setWs] = useState(null);
  const MAX_POINTS = 20000;

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
          const raw = event.data;

          // Helper to coerce a candidate object into a point or null
          const asPoint = (obj) => {
            if (!obj || typeof obj !== 'object') return null;
            // Support different key casings and numeric strings
            const x = obj.x ?? obj.X;
            const y = obj.y ?? obj.Y;
            const z = obj.z ?? obj.Z;
            const ts = obj.timestamp ?? obj.time ?? obj.t ?? obj.T;
            const px = Number(x);
            const py = Number(y);
            const pz = Number(z);
            const pt = Number(ts);
            if ([px, py, pz, pt].every((v) => Number.isFinite(v))) {
              return { x: px, y: py, z: pz, timestamp: pt };
            }
            return null;
          };

          const appendPoints = (newPts) => {
            if (!newPts || newPts.length === 0) return;
            setPoints((prev) => {
              const combined = prev.concat(newPts);
              // Cap buffer to avoid unbounded memory growth
              return combined.length > MAX_POINTS
                ? combined.slice(combined.length - MAX_POINTS)
                : combined;
            });
          };

          // Try JSON first
          try {
            const data = JSON.parse(raw);

            // Ignore known non-point messages
            if (data && typeof data === 'object' && data.type && data.type !== 'points') {
              return;
            }

            if (Array.isArray(data)) {
              appendPoints(
                data
                  .map(asPoint)
                  .filter(Boolean)
              );
              return;
            }

            if (data && typeof data === 'object') {
              if (Array.isArray(data.points)) {
                appendPoints(
                  data.points
                    .map(asPoint)
                    .filter(Boolean)
                );
                return;
              }
              const pt = asPoint(data);
              if (pt) {
                appendPoints([pt]);
                return;
              }
            }
          } catch (_) {
            // Not JSON; try CSV: "x,y,z,timestamp"
            if (typeof raw === 'string') {
              const parts = raw.split(',').map((s) => s.trim());
              if (parts.length === 4) {
                const [sx, sy, sz, st] = parts;
                const pt = {
                  x: Number(sx),
                  y: Number(sy),
                  z: Number(sz),
                  timestamp: Number(st)
                };
                if ([pt.x, pt.y, pt.z, pt.timestamp].every((v) => Number.isFinite(v))) {
                  appendPoints([pt]);
                  return;
                }
              }
            }
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
