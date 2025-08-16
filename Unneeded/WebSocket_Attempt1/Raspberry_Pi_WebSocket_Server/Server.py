#!/usr/bin/env python3
"""
Raspberry Pi WebSocket Server
Real-time data streaming to React clients
"""

import asyncio
import websockets
import json
import time
import random
from datetime import datetime
from typing import Set, Dict, Any
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Connected clients
connected_clients: Set[websockets.WebSocketServerProtocol] = set()

# Configuration - EASILY MODIFY THIS SECTION
CONFIG = {
    "host": "0.0.0.0",  # Listen on all interfaces
    "port": 8765,
    "data_interval": 1.0,  # Seconds between data broadcasts
}

class DataCollector:
    """
    Customize this class to collect your specific data
    Add your sensors, GPIO pins, or any data sources here
    """
    
    def __init__(self):
        # Initialize your sensors/hardware here
        pass
    
    def get_sensor_data(self) -> Dict[str, Any]:
        """
        MODIFY THIS METHOD to return your actual sensor data
        Current example returns simulated data
        """
        try:
            # Example data - replace with your actual sensor readings
            data = {
                "timestamp": datetime.now().isoformat(),
                "temperature": round(random.uniform(20.0, 30.0), 2),
                "humidity": round(random.uniform(40.0, 80.0), 2),
                "pressure": round(random.uniform(1000.0, 1020.0), 2),
                "light_level": random.randint(0, 1023),
                "motion_detected": random.choice([True, False]),
                "cpu_temp": self.get_cpu_temperature(),
                "system_load": self.get_system_load()
            }
            
            # Add your custom data here
            # data["custom_sensor"] = your_sensor_reading()
            
            return data
            
        except Exception as e:
            logger.error(f"Error collecting sensor data: {e}")
            return {"error": str(e), "timestamp": datetime.now().isoformat()}
    
    def get_cpu_temperature(self) -> float:
        """Get Raspberry Pi CPU temperature"""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = float(f.read()) / 1000.0
                return round(temp, 2)
        except:
            return 0.0
    
    def get_system_load(self) -> float:
        """Get system load average"""
        try:
            with open('/proc/loadavg', 'r') as f:
                load = float(f.read().split()[0])
                return round(load, 2)
        except:
            return 0.0

class WebSocketServer:
    def __init__(self):
        self.data_collector = DataCollector()
        self.running = False
    
    async def register_client(self, websocket):
        """Register a new client"""
        connected_clients.add(websocket)
        client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        logger.info(f"Client connected: {client_info} (Total: {len(connected_clients)})")
        
        # Send welcome message
        welcome_msg = {
            "type": "connection",
            "status": "connected",
            "message": "Connected to Raspberry Pi WebSocket server",
            "server_time": datetime.now().isoformat()
        }
        await websocket.send(json.dumps(welcome_msg))
    
    async def unregister_client(self, websocket):
        """Unregister a client"""
        connected_clients.discard(websocket)
        client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        logger.info(f"Client disconnected: {client_info} (Total: {len(connected_clients)})")
    
    async def handle_client_message(self, websocket, message):
        """
        Handle incoming messages from clients
        MODIFY THIS to handle custom commands
        """
        try:
            data = json.loads(message)
            msg_type = data.get("type", "unknown")
            
            if msg_type == "ping":
                # Respond to ping
                response = {"type": "pong", "timestamp": datetime.now().isoformat()}
                await websocket.send(json.dumps(response))
            
            elif msg_type == "get_data":
                # Send current sensor data immediately
                sensor_data = self.data_collector.get_sensor_data()
                response = {"type": "sensor_data", "data": sensor_data}
                await websocket.send(json.dumps(response))
            
            elif msg_type == "command":
                # Handle custom commands
                command = data.get("command", "")
                result = await self.handle_command(command, data.get("params", {}))
                response = {"type": "command_result", "command": command, "result": result}
                await websocket.send(json.dumps(response))
            
            else:
                logger.warning(f"Unknown message type: {msg_type}")
        
        except json.JSONDecodeError:
            logger.error(f"Invalid JSON received: {message}")
        except Exception as e:
            logger.error(f"Error handling client message: {e}")
    
    async def handle_command(self, command: str, params: Dict) -> Dict:
        """
        Handle custom commands from clients
        ADD YOUR CUSTOM COMMANDS HERE
        """
        try:
            if command == "set_interval":
                # Change data broadcast interval
                new_interval = params.get("interval", 1.0)
                CONFIG["data_interval"] = max(0.1, min(60.0, new_interval))  # Clamp between 0.1 and 60 seconds
                return {"status": "success", "new_interval": CONFIG["data_interval"]}
            
            elif command == "restart_data":
                # Restart data collection
                return {"status": "success", "message": "Data collection restarted"}
            
            # Add your custom commands here
            # elif command == "turn_on_led":
            #     # Your LED control code
            #     return {"status": "success", "message": "LED turned on"}
            
            else:
                return {"status": "error", "message": f"Unknown command: {command}"}
        
        except Exception as e:
            return {"status": "error", "message": str(e)}
    
    async def broadcast_data(self):
        """Broadcast sensor data to all connected clients"""
        if not connected_clients:
            return
        
        try:
            sensor_data = self.data_collector.get_sensor_data()
            message = {
                "type": "sensor_data",
                "data": sensor_data,
                "client_count": len(connected_clients)
            }
            
            # Send to all connected clients
            disconnected = set()
            for client in connected_clients.copy():
                try:
                    await client.send(json.dumps(message))
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(client)
                except Exception as e:
                    logger.error(f"Error sending to client: {e}")
                    disconnected.add(client)
            
            # Remove disconnected clients
            for client in disconnected:
                connected_clients.discard(client)
            
            if len(connected_clients) > 0:
                logger.debug(f"Broadcasted data to {len(connected_clients)} clients")
        
        except Exception as e:
            logger.error(f"Error broadcasting data: {e}")
    
    async def data_broadcaster(self):
        """Background task to broadcast data at regular intervals"""
        logger.info(f"Started data broadcaster (interval: {CONFIG['data_interval']}s)")
        while self.running:
            await self.broadcast_data()
            await asyncio.sleep(CONFIG["data_interval"])
    
    async def handle_client(self, websocket, path):
        """Handle individual client connections"""
        await self.register_client(websocket)
        try:
            async for message in websocket:
                await self.handle_client_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            logger.error(f"Error in client handler: {e}")
        finally:
            await self.unregister_client(websocket)
    
    async def start_server(self):
        """Start the WebSocket server"""
        self.running = True
        
        # Start the data broadcaster task
        broadcaster_task = asyncio.create_task(self.data_broadcaster())
        
        # Start the WebSocket server
        logger.info(f"Starting WebSocket server on {CONFIG['host']}:{CONFIG['port']}")
        
        try:
            async with websockets.serve(self.handle_client, CONFIG["host"], CONFIG["port"]):
                logger.info("WebSocket server started successfully!")
                logger.info(f"Connect from React app using: ws://{CONFIG['host']}:{CONFIG['port']}")
                
                # Keep the server running
                await asyncio.Future()  # Run forever
        
        except Exception as e:
            logger.error(f"Server error: {e}")
        finally:
            self.running = False
            broadcaster_task.cancel()

def main():
    """Main entry point"""
    print(" Raspberry Pi WebSocket Server")
    print("=" * 40)
    print(f"Host: {CONFIG['host']}")
    print(f"Port: {CONFIG['port']}")
    print(f"Data interval: {CONFIG['data_interval']}s")
    print("=" * 40)
    
    server = WebSocketServer()
    
    try:
        asyncio.run(server.start_server())
    except KeyboardInterrupt:
        print("\n Server stopped by user")
    except Exception as e:
        print(f" Server error: {e}")

if __name__ == "__main__":
    main()