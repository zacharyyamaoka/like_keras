#!/usr/bin/env python3
"""
    Minimal Viewer Server
    
    Simple FastAPI server that:
    1. Serves the robot description as JSON
    2. Watches for file changes and sends updates via WebSocket
    3. Serves a simple HTML viewer
"""

# BAM
# (minimal dependencies for now)

# PYTHON
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import asyncio
import json
from pathlib import Path
from typing import Any, List, Dict
import importlib.util
import sys
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler


app = FastAPI()

# Store active WebSocket connections
active_connections: list[WebSocket] = []
node_connections: list[WebSocket] = []

# Store node state in memory
nodes_state: List[Dict[str, Any]] = []

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://127.0.0.1:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class RobotFileHandler(FileSystemEventHandler):
    """Watches for changes to the robot description file."""
    
    def __init__(self, robot_file: str, callback):
        self.robot_file = Path(robot_file).resolve()
        self.callback = callback
        
    def on_modified(self, event):
        if event.is_directory:
            return
        if Path(event.src_path).resolve() == self.robot_file:
            print(f"🔄 Detected change in {self.robot_file.name}")
            self.callback()


class ViewerServer:
    def __init__(self, robot_file: str):
        self.robot_file = Path(robot_file).resolve()
        self.robot_instance = None
        self.load_robot()
        
        # Setup file watcher
        self.observer = Observer()
        handler = RobotFileHandler(robot_file, self.on_file_change)
        self.observer.schedule(handler, str(self.robot_file.parent), recursive=False)
        self.observer.start()
        
    def load_robot(self):
        """Dynamically import and instantiate the robot."""
        try:
            # Load the module
            spec = importlib.util.spec_from_file_location("robot_module", self.robot_file)
            module = importlib.util.module_from_spec(spec)
            sys.modules["robot_module"] = module
            spec.loader.exec_module(module)
            
            # Find the robot class (assume it's the first RobotDescription subclass)
            for name in dir(module):
                obj = getattr(module, name)
                if isinstance(obj, type) and hasattr(obj, 'to_dict') and name != 'RobotDescription':
                    self.robot_instance = obj()
                    print(f"✅ Loaded robot: {name}")
                    break
                    
        except Exception as e:
            print(f"❌ Error loading robot: {e}")
            import traceback
            traceback.print_exc()
            
    def on_file_change(self):
        """Called when the robot file changes."""
        self.load_robot()
        # Notify all connected WebSocket clients
        asyncio.create_task(self.broadcast_update())
        
    async def broadcast_update(self):
        """Send update to all connected clients."""
        if self.robot_instance:
            data = self.robot_instance.to_dict()
            disconnected = []
            for connection in active_connections:
                try:
                    await connection.send_json({"type": "update", "data": data})
                except:
                    disconnected.append(connection)
            
            # Remove disconnected clients
            for conn in disconnected:
                active_connections.remove(conn)
                
    def get_robot_data(self) -> dict:
        """Get current robot data."""
        if self.robot_instance:
            return self.robot_instance.to_dict()
        return {"error": "No robot loaded"}


# Global server instance (will be initialized at startup)
viewer_server: ViewerServer = None


@app.on_event("startup")
async def startup_event():
    global viewer_server
    # Check environment variable first, then default to simple_rr_robot.py
    import os
    robot_file = os.environ.get('ROBOT_FILE')
    if not robot_file:
        robot_file = Path(__file__).parent / "simple_rr_robot.py"
    viewer_server = ViewerServer(str(robot_file))


@app.get("/api/robot")
async def get_robot():
    """Get current robot description."""
    return viewer_server.get_robot_data()


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket for live updates."""
    await websocket.accept()
    active_connections.append(websocket)
    
    # Send initial data
    await websocket.send_json({
        "type": "init",
        "data": viewer_server.get_robot_data()
    })
    
    try:
        while True:
            # Keep connection alive
            await websocket.receive_text()
    except:
        active_connections.remove(websocket)


@app.get("/api/nodes")
async def get_nodes():
    """Get current node state."""
    return {"nodes": nodes_state}


@app.post("/api/nodes")
async def update_nodes(nodes: List[Dict[str, Any]]):
    """Update node state."""
    global nodes_state
    nodes_state = nodes
    # Broadcast to all connected WebSocket clients
    await broadcast_nodes_update()
    return {"status": "ok", "nodes": nodes_state}


async def broadcast_nodes_update():
    """Broadcast node updates to all connected clients."""
    disconnected = []
    for connection in node_connections:
        try:
            await connection.send_json({
                "type": "nodes_update",
                "nodes": nodes_state,
            })
        except:
            disconnected.append(connection)
    
    # Remove disconnected clients
    for conn in disconnected:
        node_connections.remove(conn)


@app.websocket("/ws/nodes")
async def nodes_websocket_endpoint(websocket: WebSocket):
    """WebSocket for node updates."""
    global nodes_state
    await websocket.accept()
    node_connections.append(websocket)
    
    # Send initial state
    await websocket.send_json({
        "type": "init",
        "nodes": nodes_state,
    })
    
    try:
        while True:
            data = await websocket.receive_text()
            message = json.loads(data)
            
            if message.get("type") == "get_nodes":
                # Send current nodes
                await websocket.send_json({
                    "type": "nodes_update",
                    "nodes": nodes_state,
                })
            elif message.get("type") == "nodes_update":
                # Update nodes and broadcast to all clients
                nodes_state = message.get("nodes", [])
                await broadcast_nodes_update()
    except WebSocketDisconnect:
        node_connections.remove(websocket)
    except Exception as e:
        print(f"WebSocket error: {e}")
        if websocket in node_connections:
            node_connections.remove(websocket)


@app.get("/", response_class=HTMLResponse)
async def get_viewer():
    """Serve the HTML viewer."""
    html_file = Path(__file__).parent / "viewer.html"
    if html_file.exists():
        return html_file.read_text()
    return """
    <html>
        <head><title>LK Viewer</title></head>
        <body>
            <h1>LK Viewer</h1>
            <p>viewer.html not found. Please create it.</p>
        </body>
    </html>
    """


if __name__ == "__main__":
    print("🚀 Starting LK Viewer Server...")
    print("📍 Open http://localhost:8000 in your browser")
    uvicorn.run(app, host="0.0.0.0", port=8000)

