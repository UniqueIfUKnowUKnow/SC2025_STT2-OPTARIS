import React, { useRef, useEffect, useState } from 'react';
import * as THREE from 'three';
import './App.css';

// Type definitions for the data points
interface DataPoint {
    x: number;
    y: number;
    z: number;
    timestamp: number;
}

// Main App component
const App: React.FC = () => {
    // Refs for the canvas and message box elements
    const containerRef = useRef<HTMLDivElement>(null);
    const messageBoxRef = useRef<HTMLDivElement>(null);

    // State to hold the data points and WebSocket connection status
    const [dataPoints, setDataPoints] = useState<DataPoint[]>([]);
    const [isConnected, setIsConnected] = useState(false);

    // Refs for Three.js objects to maintain their state across renders
    const sceneRef = useRef<THREE.Scene | null>(null);
    const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
    const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
    const lineRef = useRef<THREE.Line | null>(null);

    // WebSocket connection handler
    useEffect(() => {
        // Candidate URLs for the WebSocket server (fallbacks if 8080 is busy)
        const wsUrls = ['ws://localhost:8080','ws://localhost:8081','ws://localhost:8082','ws://localhost:8083'];
        let ws: WebSocket;
        let currentIndex = 0;

        // Function to connect to the WebSocket
        const connectWebSocket = () => {
            const url = wsUrls[currentIndex % wsUrls.length];
            console.log(`Attempting WS connection to ${url}`);
            ws = new WebSocket(url);

            ws.onopen = () => {
                console.log('Connected to WebSocket server');
                setIsConnected(true);
                // Display connection status in the message box
                showMessage('Connection established!', 'success');
            };

            ws.onmessage = (event) => {
                try {
                    // Parse the incoming data
                    const data: DataPoint = JSON.parse(event.data);
                    console.log('Received data:', data);

                    // Update the state with the new data point
                    setDataPoints(prevPoints => {
                        const newPoints = [...prevPoints, data];
                        // Only keep the last 500 points to prevent performance issues
                        if (newPoints.length > 500) {
                            newPoints.shift();
                        }
                        return newPoints;
                    });
                } catch (error) {
                    console.error('Failed to parse incoming data:', error);
                    showMessage('Failed to parse incoming data!', 'error');
                }
            };

            ws.onclose = () => {
                console.log('Disconnected from WebSocket server');
                setIsConnected(false);
                showMessage('Disconnected. Retrying in 5 seconds...', 'error');
                // Attempt to reconnect after a delay
                // Try next URL on each reconnect attempt
                currentIndex = (currentIndex + 1) % wsUrls.length;
                setTimeout(connectWebSocket, 2000);
            };

            ws.onerror = (error) => {
                console.error('WebSocket error:', error);
                ws.close();
            };
        };

        connectWebSocket();

        // Clean up function to close the WebSocket connection
        return () => {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.close();
            }
        };
    }, []);

    // Three.js scene setup
    useEffect(() => {
        // Wait for the container element to be available
        if (!containerRef.current) return;

        // Scene setup
        const scene = new THREE.Scene();
        sceneRef.current = scene;

        // Camera setup
        const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        camera.position.z = 500;
        cameraRef.current = camera;

        // Renderer setup
        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(window.innerWidth, window.innerHeight);
        rendererRef.current = renderer;
        containerRef.current.appendChild(renderer.domElement);

        // Add a central point to the scene
        const centralGeometry = new THREE.SphereGeometry(2, 32, 32);
        const centralMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff });
        const centralSphere = new THREE.Mesh(centralGeometry, centralMaterial);
        scene.add(centralSphere);

        // Add ambient light
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        scene.add(ambientLight);

        // Animation loop
        const animate = () => {
            requestAnimationFrame(animate);
            renderer.render(scene, camera);
        };

        // Handle window resize
        const handleResize = () => {
            const width = window.innerWidth;
            const height = window.innerHeight;
            camera.aspect = width / height;
            camera.updateProjectionMatrix();
            renderer.setSize(width, height);
        };

        window.addEventListener('resize', handleResize);

        animate();

        // Clean up Three.js objects
        return () => {
            window.removeEventListener('resize', handleResize);
            renderer.dispose();
            // Remove the canvas element from the DOM
            if (containerRef.current && renderer.domElement) {
                containerRef.current.removeChild(renderer.domElement);
            }
        };
    }, []);

    // Update the scene when data points change
    useEffect(() => {
        if (!sceneRef.current || !rendererRef.current) return;

        // Remove the old line and points
        if (lineRef.current) {
            sceneRef.current.remove(lineRef.current);
            lineRef.current.geometry.dispose();
            (lineRef.current.material as THREE.Material).dispose();
        }

        // Create a new line and points if we have data
        if (dataPoints.length > 1) {
            const points = dataPoints.map(point => new THREE.Vector3(point.x, point.y, point.z));
            const geometry = new THREE.BufferGeometry().setFromPoints(points);
            const material = new THREE.LineBasicMaterial({ color: 0x00ff00 });
            const line = new THREE.Line(geometry, material);
            sceneRef.current.add(line);
            lineRef.current = line;

            // Add a small sphere for the latest point
            if (dataPoints.length > 0) {
                const latestPoint = dataPoints[dataPoints.length - 1];
                const pointGeometry = new THREE.SphereGeometry(1, 16, 16);
                const pointMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
                const pointMesh = new THREE.Mesh(pointGeometry, pointMaterial);
                pointMesh.position.set(latestPoint.x, latestPoint.y, latestPoint.z);

                // Remove the previous latest point to avoid clutter
                const oldLatest = sceneRef.current.children.find((child: THREE.Object3D) => (child as THREE.Mesh).geometry instanceof THREE.SphereGeometry && child !== sceneRef.current?.children[0]);
                if (oldLatest) {
                    sceneRef.current.remove(oldLatest);
                    oldLatest.geometry.dispose();
                    (oldLatest.material as THREE.Material).dispose();
                }

                sceneRef.current.add(pointMesh);
            }

        }
    }, [dataPoints]);

    // Function to show messages in the message box
    const showMessage = (message: string, type: 'success' | 'error' | 'info') => {
        if (messageBoxRef.current) {
            messageBoxRef.current.textContent = message;
            messageBoxRef.current.className = `message-box ${type}`;
        }
    };

    return (
        <div className="App">
            <div className="status-container">
                <div className={`connection-status ${isConnected ? 'connected' : 'disconnected'}`}>
                    {isConnected ? 'Connected' : 'Disconnected'}
                </div>
                <div ref={messageBoxRef} className="message-box info">
                    Waiting for connection...
                </div>
            </div>
            <div ref={containerRef} className="canvas-container"></div>
        </div>
    );
};

export default App;
