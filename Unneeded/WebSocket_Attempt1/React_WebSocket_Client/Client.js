import React, { useState, useEffect, useRef } from 'react';
import { Activity, Wifi, WifiOff, Thermometer, Droplets, Gauge, Sun, AlertTriangle, Cpu, BarChart3 } from 'lucide-react';

const WebSocketClient = () => {
  // WebSocket connection state
  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('Disconnected');
  const [sensorData, setSensorData] = useState(null);
  const [lastUpdate, setLastUpdate] = useState(null);
  const [clientCount, setClientCount] = useState(0);
  const [messages, setMessages] = useState([]);
  
  // Configuration - EASILY MODIFY THESE
  const WS_URL = 'ws://192.168.1.100:8765'; // Change to your Pi's IP
  const MAX_MESSAGES = 50;
  
  const wsRef = useRef(null);
  const reconnectTimeoutRef = useRef(null);

  // Custom hook for WebSocket connection
  useEffect(() => {
    connectWebSocket();
    
    return () => {
      if (wsRef.current) {
        wsRef.current.close();
      }
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
      }
    };
  }, []);

  const connectWebSocket = () => {
    try {
      setConnectionStatus('Connecting...');
      const ws = new WebSocket(WS_URL);
      
      ws.onopen = () => {
        console.log('WebSocket connected');
        setIsConnected(true);
        setConnectionStatus('Connected');
        addMessage('Connected to Raspberry Pi server', 'success');
      };
      
      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          handleIncomingMessage(data);
        } catch (error) {
          console.error('Error parsing WebSocket message:', error);
          addMessage('Error parsing server message', 'error');
        }
      };
      
      ws.onclose = () => {
        console.log('WebSocket disconnected');
        setIsConnected(false);
        setConnectionStatus('Disconnected');
        addMessage('Disconnected from server', 'warning');
        
        // Auto-reconnect after 3 seconds
        reconnectTimeoutRef.current = setTimeout(() => {
          addMessage('Attempting to reconnect...', 'info');
          connectWebSocket();
        }, 3000);
      };
      
      ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        setConnectionStatus('Connection Error');
        addMessage('WebSocket connection error', 'error');
      };
      
      wsRef.current = ws;
    } catch (error) {
      console.error('Failed to create WebSocket connection:', error);
      setConnectionStatus('Failed to Connect');
      addMessage('Failed to create connection', 'error');
    }
  };

  const handleIncomingMessage = (data) => {
    const { type } = data;
    
    switch (type) {
      case 'sensor_data':
        setSensorData(data.data);
        setLastUpdate(new Date().toLocaleTimeString());
        setClientCount(data.client_count || 0);
        break;
        
      case 'connection':
        addMessage(data.message || 'Connected', 'success');
        break;
        
      case 'pong':
        addMessage('Pong received', 'info');
        break;
        
      case 'command_result':
        addMessage(`Command "${data.command}": ${data.result.message || 'Success'}`, 
                  data.result.status === 'success' ? 'success' : 'error');
        break;
        
      default:
        console.log('Unknown message type:', type, data);
    }
  };

  const addMessage = (message, type = 'info') => {
    const newMessage = {
      id: Date.now(),
      text: message,
      type,
      timestamp: new Date().toLocaleTimeString()
    };
    
    setMessages(prev => [newMessage, ...prev.slice(0, MAX_MESSAGES - 1)]);
  };

  // EASILY MODIFY THESE FUNCTIONS TO SEND CUSTOM DATA
  const sendMessage = (message) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(message));
    } else {
      addMessage('Cannot send: Not connected', 'error');
    }
  };

  const sendPing = () => {
    sendMessage({ type: 'ping' });
    addMessage('Ping sent', 'info');
  };

  const requestData = () => {
    sendMessage({ type: 'get_data' });
    addMessage('Data request sent', 'info');
  };

  const changeInterval = (interval) => {
    sendMessage({ 
      type: 'command', 
      command: 'set_interval', 
      params: { interval: parseFloat(interval) }
    });
    addMessage(`Requested interval change to ${interval}s`, 'info');
  };

  // ADD YOUR CUSTOM COMMANDS HERE
  const sendCustomCommand = (command, params = {}) => {
    sendMessage({ 
      type: 'command', 
      command, 
      params 
    });
    addMessage(`Custom command sent: ${command}`, 'info');
  };

  const getStatusColor = () => {
    if (isConnected) return 'text-green-500';
    if (connectionStatus === 'Connecting...') return 'text-yellow-500';
    return 'text-red-500';
  };

  const getMessageColor = (type) => {
    const colors = {
      success: 'text-green-600',
      error: 'text-red-600',
      warning: 'text-yellow-600',
      info: 'text-blue-600'
    };
    return colors[type] || 'text-gray-600';
  };

  return (
    <div className="min-h-screen bg-gray-100 p-6">
      <div className="max-w-6xl mx-auto">
        {/* Header */}
        <div className="bg-white rounded-lg shadow-md p-6 mb-6">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-3">
              <Activity className="w-8 h-8 text-blue-500" />
              <div>
                <h1 className="text-2xl font-bold text-gray-800">
                  Raspberry Pi Dashboard
                </h1>
                <p className="text-gray-600">Real-time sensor monitoring</p>
              </div>
            </div>
            
            <div className="flex items-center gap-4">
              <div className="flex items-center gap-2">
                {isConnected ? (
                  <Wifi className={`w-5 h-5 ${getStatusColor()}`} />
                ) : (
                  <WifiOff className={`w-5 h-5 ${getStatusColor()}`} />
                )}
                <span className={`font-medium ${getStatusColor()}`}>
                  {connectionStatus}
                </span>
              </div>
              
              {clientCount > 0 && (
                <div className="text-sm text-gray-500">
                  {clientCount} client{clientCount !== 1 ? 's' : ''} connected
                </div>
              )}
              
              {lastUpdate && (
                <div className="text-sm text-gray-500">
                  Last update: {lastUpdate}
                </div>
              )}
            </div>
          </div>
        </div>

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
          {/* Sensor Data Display */}
          <div className="lg:col-span-2">
            <div className="bg-white rounded-lg shadow-md p-6">
              <h2 className="text-xl font-semibold text-gray-800 mb-4">
                Live Sensor Data
              </h2>
              
              {sensorData ? (
                <div className="grid grid-cols-2 md:grid-cols-3 gap-4">
                  {/* Temperature */}
                  <div className="bg-red-50 p-4 rounded-lg">
                    <div className="flex items-center gap-2 mb-2">
                      <Thermometer className="w-5 h-5 text-red-500" />
                      <span className="font-medium text-gray-700">Temperature</span>
                    </div>
                    <div className="text-2xl font-bold text-red-600">
                      {sensorData.temperature}°C
                    </div>
                  </div>
                  
                  {/* Humidity */}
                  <div className="bg-blue-50 p-4 rounded-lg">
                    <div className="flex items-center gap-2 mb-2">
                      <Droplets className="w-5 h-5 text-blue-500" />
                      <span className="font-medium text-gray-700">Humidity</span>
                    </div>
                    <div className="text-2xl font-bold text-blue-600">
                      {sensorData.humidity}%
                    </div>
                  </div>
                  
                  {/* Pressure */}
                  <div className="bg-green-50 p-4 rounded-lg">
                    <div className="flex items-center gap-2 mb-2">
                      <Gauge className="w-5 h-5 text-green-500" />
                      <span className="font-medium text-gray-700">Pressure</span>
                    </div>
                    <div className="text-2xl font-bold text-green-600">
                      {sensorData.pressure} hPa
                    </div>
                  </div>
                  
                  {/* Light Level */}
                  <div className="bg-yellow-50 p-4 rounded-lg">
                    <div className="flex items-center gap-2 mb-2">
                      <Sun className="w-5 h-5 text-yellow-500" />
                      <span className="font-medium text-gray-700">Light</span>
                    </div>
                    <div className="text-2xl font-bold text-yellow-600">
                      {sensorData.light_level}
                    </div>
                  </div>
                  
                  {/* Motion */}
                  <div className="bg-purple-50 p-4 rounded-lg">
                    <div className="flex items-center gap-2 mb-2">
                      <AlertTriangle className="w-5 h-5 text-purple-500" />
                      <span className="font-medium text-gray-700">Motion</span>
                    </div>
                    <div className={`text-2xl font-bold ${
                      sensorData.motion_detected ? 'text-red-600' : 'text-green-600'
                    }`}>
                      {sensorData.motion_detected ? 'Detected' : 'Clear'}
                    </div>
                  </div>
                  
                  {/* CPU Temp */}
                  <div className="bg-orange-50 p-4 rounded-lg">
                    <div className="flex items-center gap-2 mb-2">
                      <Cpu className="w-5 h-5 text-orange-500" />
                      <span className="font-medium text-gray-700">CPU Temp</span>
                    </div>
                    <div className="text-2xl font-bold text-orange-600">
                      {sensorData.cpu_temp}°C
                    </div>
                  </div>
                </div>
              ) : (
                <div className="text-center py-12 text-gray-500">
                  <Activity className="w-12 h-12 mx-auto mb-4 opacity-50" />
                  <p>Waiting for sensor data...</p>
                  <p className="text-sm mt-2">
                    {isConnected ? 'Connected but no data received yet' : 'Not connected to server'}
                  </p>
                </div>
              )}
            </div>
          </div>

          {/* Controls and Messages */}
          <div className="space-y-6">
            {/* Control Panel */}
            <div className="bg-white rounded-lg shadow-md p-6">
              <h3 className="text-lg font-semibold text-gray-800 mb-4">
                Controls
              </h3>
              
              <div className="space-y-3">
                <button
                  onClick={sendPing}
                  disabled={!isConnected}
                  className="w-full bg-blue-500 text-white py-2 px-4 rounded disabled:bg-gray-300 hover:bg-blue-600 transition-colors"
                >
                  Send Ping
                </button>
                
                <button
                  onClick={requestData}
                  disabled={!isConnected}
                  className="w-full bg-green-500 text-white py-2 px-4 rounded disabled:bg-gray-300 hover:bg-green-600 transition-colors"
                >
                  Request Data Now
                </button>
                
                <div className="space-y-2">
                  <label className="text-sm font-medium text-gray-700">
                    Update Interval (seconds)
                  </label>
                  <select
                    onChange={(e) => changeInterval(e.target.value)}
                    disabled={!isConnected}
                    className="w-full p-2 border border-gray-300 rounded disabled:bg-gray-100"
                    defaultValue="1"
                  >
                    <option value="0.5">0.5s</option>
                    <option value="1">1s</option>
                    <option value="2">2s</option>
                    <option value="5">5s</option>
                    <option value="10">10s</option>
                  </select>
                </div>
                
                {/* ADD YOUR CUSTOM CONTROLS HERE */}
                <button
                  onClick={() => sendCustomCommand('restart_data')}
                  disabled={!isConnected}
                  className="w-full bg-orange-500 text-white py-2 px-4 rounded disabled:bg-gray-300 hover:bg-orange-600 transition-colors"
                >
                  Restart Data Collection
                </button>
              </div>
            </div>

            {/* Message Log */}
            <div className="bg-white rounded-lg shadow-md p-6">
              <h3 className="text-lg font-semibold text-gray-800 mb-4">
                Activity Log
              </h3>
              
              <div className="space-y-2 max-h-96 overflow-y-auto">
                {messages.length > 0 ? (
                  messages.map((msg) => (
                    <div key={msg.id} className="text-sm">
                      <span className="text-gray-400">{msg.timestamp}</span>{' '}
                      <span className={getMessageColor(msg.type)}>{msg.text}</span>
                    </div>
                  ))
                ) : (
                  <p className="text-gray-500 text-sm">No messages yet</p>
                )}
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default WebSocketClient;