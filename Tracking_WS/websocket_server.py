import asyncio
import websockets
import json
from coordinate_transfer import CoordinateTransfer
from trajectory_prediction import TrajectoryPredictor

class WebSocketServer:
    def __init__(self):
        self.coordinate_transfer = CoordinateTransfer()
        self.trajectory_predictor = TrajectoryPredictor()
        self.connected_clients = set()

    async def handle_client(self, websocket):
        try:
            self.connected_clients.add(websocket)
            print("Client connected")
            
            async for message in websocket:
                try:
                    data = json.loads(message)
                    command = data.get('command')
                    
                    if command == 'get_coordinates':
                        # Get current coordinates and send them back
                        coordinates = self.coordinate_transfer.get_current_coordinates()
                        await websocket.send(json.dumps({
                            'type': 'coordinates',
                            'data': coordinates
                        }))
                    
                    elif command == 'get_prediction':
                        # Get trajectory prediction
                        prediction = self.trajectory_predictor.predict_next_position()
                        await websocket.send(json.dumps({
                            'type': 'prediction',
                            'data': prediction
                        }))
                
                except json.JSONDecodeError:
                    print("Invalid JSON received")
                
        except websockets.exceptions.ConnectionClosed:
            print("Client disconnected")
        finally:
            self.connected_clients.remove(websocket)

    async def broadcast_updates(self):
        while True:
            if self.connected_clients:
                coordinates = self.coordinate_transfer.get_current_coordinates()
                message = json.dumps({
                    'type': 'coordinates',
                    'data': coordinates
                })
                
                # Broadcast to all connected clients
                websockets.broadcast(self.connected_clients, message)
            
            await asyncio.sleep(0.1)  # Update every 100ms

    async def start_server(self):
        server = await websockets.serve(
            self.handle_client,
            "0.0.0.0",  # Listen on all network interfaces
            8765  # Port number
        )
        
        # Start the broadcast task
        asyncio.create_task(self.broadcast_updates())
        
        print("WebSocket server started on ws://0.0.0.0:8765")
        await server.wait_closed()

if __name__ == "__main__":
    server = WebSocketServer()
    asyncio.run(server.start_server())
