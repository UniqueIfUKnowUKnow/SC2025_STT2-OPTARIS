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
    const sceneRef = useRef<any>(null);
    const cameraRef = useRef<any>(null);
    const rendererRef = useRef<any>(null);
    const lineRef = useRef<any>(null);

    // WebSocket connection handler
    useEffect(() => {
        // Candidate URLs for the WebSocket server (fallbacks if 8080 is busy)
        const wsUrls = ['ws://192.168.55.126:8080','ws://192.168.55.126:8081','ws://192.168.55.126:8082','ws://192.168.55.126:8083'];
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

        // Simple orbit-style pointer controls with pointer capture
        const target = new THREE.Vector3(0, 0, 0);
        const spherical = new THREE.Spherical();
        spherical.setFromVector3(camera.position.clone().sub(target));
        let isDragging = false;
        let lastX = 0;
        let lastY = 0;

        // Improve UX on the canvas
        renderer.domElement.style.touchAction = 'none';
        (renderer.domElement as any).style.pointerEvents = 'auto';
        renderer.domElement.style.cursor = 'grab';

        const onPointerDown = (event: PointerEvent) => {
            isDragging = true;
            lastX = event.clientX;
            lastY = event.clientY;
            renderer.domElement.setPointerCapture(event.pointerId);
            renderer.domElement.style.cursor = 'grabbing';
            console.log('Drag start');
        };

        const onPointerMove = (event: PointerEvent) => {
            if (!isDragging) return;
            const deltaX = event.clientX - lastX;
            const deltaY = event.clientY - lastY;
            lastX = event.clientX;
            lastY = event.clientY;

            const rotateSpeed = 0.005;
            spherical.theta -= deltaX * rotateSpeed;
            spherical.phi -= deltaY * rotateSpeed;
            const EPS = 0.000001;
            spherical.phi = Math.max(EPS, Math.min(Math.PI - EPS, spherical.phi));

            const newPos = new THREE.Vector3().setFromSpherical(spherical).add(target);
            camera.position.copy(newPos);
            camera.lookAt(target);
        };

        const onPointerUp = (event: PointerEvent) => {
            isDragging = false;
            try { renderer.domElement.releasePointerCapture(event.pointerId); } catch {}
            renderer.domElement.style.cursor = 'grab';
        };

        const onWheel = (event: WheelEvent) => {
            event.preventDefault();
            const zoomFactor = 1 + (event.deltaY > 0 ? 0.1 : -0.1);
            spherical.radius *= zoomFactor;
            spherical.radius = Math.max(10, Math.min(2000, spherical.radius));
            const newPos = new THREE.Vector3().setFromSpherical(spherical).add(target);
            camera.position.copy(newPos);
            camera.lookAt(target);
        };

        renderer.domElement.addEventListener('pointerdown', onPointerDown);
        window.addEventListener('pointermove', onPointerMove);
        window.addEventListener('pointerup', onPointerUp);
        renderer.domElement.addEventListener('wheel', onWheel, { passive: false });

        // Add a central cube for reference
        const cubeSize = 10;
        const cubeGeometry = new THREE.BoxGeometry(cubeSize, cubeSize, cubeSize);
        const cubeMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true });
        const centralCube = new THREE.Mesh(cubeGeometry, cubeMaterial);
        scene.add(centralCube);

        // Add arrows to indicate positive X (red), Y (green), and Z (blue) axes
        const origin = new THREE.Vector3(0, 0, 0);
        const axisLength = 80;
        const headLength = 16;
        const headWidth = 8;
        const arrowX = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), origin, axisLength, 0xff0000, headLength, headWidth);
        const arrowY = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), origin, axisLength, 0x00ff00, headLength, headWidth);
        const arrowZ = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), origin, axisLength, 0x0000ff, headLength, headWidth);
        scene.add(arrowX);
        scene.add(arrowY);
        scene.add(arrowZ);

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
            renderer.domElement.removeEventListener('pointerdown', onPointerDown as any);
            window.removeEventListener('pointermove', onPointerMove as any);
            window.removeEventListener('pointerup', onPointerUp as any);
            renderer.domElement.removeEventListener('wheel', onWheel as any);
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
            (lineRef.current.material as any)?.dispose?.();
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
                const oldLatest = sceneRef.current.children.find((child: any) => (child as any).geometry instanceof THREE.SphereGeometry && child !== sceneRef.current?.children[0]);
                if (oldLatest) {
                    sceneRef.current.remove(oldLatest);
                    (oldLatest as any).geometry?.dispose?.();
                    (oldLatest as any).material?.dispose?.();
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
