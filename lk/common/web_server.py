#!/usr/bin/env python3

"""
    Web server component for system visualization and monitoring.
    
    Automatically connects to all component state and diagnostics ports
    to provide real-time visualization in the browser.
"""

# BAM
from lk.common.component import LifecycleComponent
from lk.common.port import InputPort
from lk.msgs.diagnostics import ComponentStateMsg, DiagnosticStatus, SystemDiagnostics, DiagnosticLevel
from lk.common.system import System

# PYTHON
from typing import Dict, List, Optional, Set
from dataclasses import dataclass
import time
from collections import defaultdict

# FastAPI for web server
try:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse
    import uvicorn
    import asyncio
    FASTAPI_AVAILABLE = True
except ImportError:
    FASTAPI_AVAILABLE = False
    asyncio = None


@dataclass
class WebServerConfig:
    """Configuration for web server component."""
    host: str = "0.0.0.0"
    port: int = 8080
    auto_connect_all: bool = True  # Auto-connect to all state/diagnostics ports
    excluded_components: List[str] = None  # Components to exclude from monitoring
    update_rate_hz: float = 10.0  # Update rate for diagnostics aggregation


class WebServerComponent(LifecycleComponent):
    """
        Web server component for system monitoring.
        
        By default, automatically connects to all component state and
        diagnostics ports. Provides real-time visualization via WebSocket.
    """
    
    Config = WebServerConfig
    
    def __init__(self, name: str = "web_server", config: Optional[WebServerConfig] = None, **kwargs):
        """Initialize web server component."""
        super().__init__(name=name, config=config or WebServerConfig(), **kwargs)
        
        if not FASTAPI_AVAILABLE:
            raise ImportError(
                "FastAPI and uvicorn required for WebServerComponent. "
                "Install with: pip install fastapi uvicorn websockets"
            )
        
        # Store component states and diagnostics
        self._component_states: Dict[str, ComponentStateMsg] = {}
        self._component_diagnostics: Dict[str, DiagnosticStatus] = {}
        self._last_update_time = time.time()
        
        # WebSocket connections
        self._websocket_connections: List[WebSocket] = []
        
        # Diagnostic propagation tracking
        self._diagnostic_propagation: Dict[str, List[str]] = defaultdict(list)
        
        # FastAPI app
        self.app = FastAPI(title="System Monitor")
        self._setup_routes()
    
    def _setup_routes(self):
        """Setup FastAPI routes."""
        
        @self.app.get("/")
        async def index():
            """Serve main visualization page."""
            html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>System Monitor</title>
                <style>
                    body { font-family: Arial, sans-serif; margin: 20px; }
                    .component { border: 1px solid #ccc; margin: 10px; padding: 10px; }
                    .ok { background-color: #d4edda; }
                    .warn { background-color: #fff3cd; }
                    .error { background-color: #f8d7da; }
                    .stale { background-color: #e2e3e5; }
                </style>
            </head>
            <body>
                <h1>System Monitor</h1>
                <div id="status">Connecting...</div>
                <div id="components"></div>
                <script>
                    const ws = new WebSocket(`ws://${window.location.host}/ws`);
                    const statusEl = document.getElementById('status');
                    const componentsEl = document.getElementById('components');
                    
                    ws.onopen = () => {
                        statusEl.textContent = 'Connected ✅';
                        statusEl.className = 'ok';
                    };
                    
                    ws.onclose = () => {
                        statusEl.textContent = 'Disconnected ❌';
                        statusEl.className = 'error';
                    };
                    
                    ws.onmessage = (event) => {
                        const data = JSON.parse(event.data);
                        updateDisplay(data);
                    };
                    
                    function updateDisplay(diagnostics) {
                        componentsEl.innerHTML = '';
                        
                        for (const [name, state] of Object.entries(diagnostics.component_states)) {
                            const diag = diagnostics.component_diagnostics[name] || {level: 'ok'};
                            const div = document.createElement('div');
                            div.className = `component ${diag.level}`;
                            div.innerHTML = `
                                <h3>${name}</h3>
                                <p>State: ${state.state}</p>
                                <p>Diagnostic: ${diag.level} - ${diag.message || 'OK'}</p>
                                ${diag.hardware_id ? `<p>Hardware ID: ${diag.hardware_id}</p>` : ''}
                            `;
                            componentsEl.appendChild(div);
                        }
                    }
                </script>
            </body>
            </html>
            """
            return HTMLResponse(html)
        
        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            """WebSocket endpoint for real-time updates."""
            await websocket.accept()
            self._websocket_connections.append(websocket)
            
            try:
                # Send initial state
                diagnostics = self._aggregate_diagnostics()
                await websocket.send_json(diagnostics.to_dict())
                
                # Keep connection alive and send updates
                while True:
                    await websocket.receive_text()
                    diagnostics = self._aggregate_diagnostics()
                    await websocket.send_json(diagnostics.to_dict())
            except WebSocketDisconnect:
                self._websocket_connections.remove(websocket)
    
    def on_configure(self):
        """Start FastAPI server."""
        import threading
        
        def run_server():
            uvicorn.run(
                self.app,
                host=self.config.host,
                port=self.config.port,
                log_level="info"
            )
        
        self._server_thread = threading.Thread(target=run_server, daemon=True)
        self._server_thread.start()
    
    def on_activate(self):
        """Auto-connect to all component ports if enabled."""
        if self.config.auto_connect_all:
            self._auto_connect_ports()
    
    def _auto_connect_ports(self):
        """Automatically connect to all component state and diagnostics ports."""
        # Get the system this component belongs to
        system = self._find_system()
        if not system:
            return
        
        excluded = set(self.config.excluded_components or [])
        
        # Find all components with state/diagnostics ports
        for component in system._components:
            if component.name in excluded or component == self:
                continue
            
            # Try to connect to state port
            if hasattr(component, 'state_port'):
                state_input = InputPort(
                    name=f"{component.name}_state",
                    msg_type=ComponentStateMsg,
                    owner=self
                )
                self.inputs.add_port(f"{component.name}_state", state_input)
                system.graph.connect(component.state_port, state_input)
            
            # Try to connect to diagnostics port
            if hasattr(component, 'diagnostics_port'):
                diag_input = InputPort(
                    name=f"{component.name}_diagnostics",
                    msg_type=DiagnosticStatus,
                    owner=self
                )
                self.inputs.add_port(f"{component.name}_diagnostics", diag_input)
                system.graph.connect(component.diagnostics_port, diag_input)
    
    def _find_system(self) -> Optional[System]:
        """Find the System this component belongs to."""
        # Walk up the ownership chain
        current = self
        while current:
            if isinstance(current, System):
                return current
            if hasattr(current, 'node') and current.node:
                current = current.node
            else:
                break
        return None
    
    def step(self):
        """Update diagnostics from connected ports."""
        # Read all state and diagnostics ports
        for port_name, port in self.inputs._ports.items():
            if port.value is None:
                continue
            
            if port_name.endswith("_state"):
                component_name = port_name[:-6]  # Remove "_state" suffix
                self._component_states[component_name] = port.value
            elif port_name.endswith("_diagnostics"):
                component_name = port_name[:-12]  # Remove "_diagnostics" suffix
                self._component_diagnostics[component_name] = port.value
        
        # Update propagation graph
        self._update_propagation()
        
        # Broadcast to WebSocket clients
        if self._websocket_connections and asyncio:
            diagnostics = self._aggregate_diagnostics()
            for ws in self._websocket_connections:
                try:
                    # Use asyncio.run_coroutine_threadsafe for thread-safe async calls
                    loop = asyncio.get_event_loop()
                    if loop.is_running():
                        asyncio.create_task(ws.send_json(diagnostics.to_dict()))
                    else:
                        loop.run_until_complete(ws.send_json(diagnostics.to_dict()))
                except:
                    pass
    
    def _update_propagation(self):
        """Update diagnostic propagation through the graph."""
        system = self._find_system()
        if not system:
            return
        
        self._diagnostic_propagation.clear()
        
        # For each component with an error, find downstream components
        for comp_name, diag in self._component_diagnostics.items():
            if diag.level == DiagnosticLevel.ERROR:
                # Find component
                component = None
                for comp in system._components:
                    if comp.name == comp_name:
                        component = comp
                        break
                
                if component:
                    # Find all downstream components via graph traversal
                    affected = self._find_downstream_components(system, component)
                    self._diagnostic_propagation[comp_name] = [c.name for c in affected]
                    
                    # Mark affected components in diagnostics
                    for affected_comp in affected:
                        if affected_comp.name in self._component_diagnostics:
                            self._component_diagnostics[affected_comp.name].affected_components.append(comp_name)
    
    def _find_downstream_components(self, system, component) -> List:
        """Find all components downstream from this one."""
        downstream = []
        visited = set()
        
        def traverse(comp):
            if comp in visited:
                return
            visited.add(comp)
            
            # Find all connections from this component's outputs
            for conn in system.graph.connections:
                if conn.source.owner == comp:
                    target_comp = conn.target.owner
                    if target_comp and target_comp not in visited:
                        downstream.append(target_comp)
                        traverse(target_comp)
        
        traverse(component)
        return downstream
    
    def _aggregate_diagnostics(self) -> SystemDiagnostics:
        """Aggregate all diagnostics into system-wide message."""
        system = self._find_system()
        system_name = system.config.name if system else "unknown"
        
        # Determine overall system health
        system_health = DiagnosticLevel.OK
        critical_errors = []
        
        for comp_name, diag in self._component_diagnostics.items():
            if diag.level == DiagnosticLevel.ERROR:
                system_health = DiagnosticLevel.ERROR
                critical_errors.append(comp_name)
            elif diag.level == DiagnosticLevel.WARN and system_health == DiagnosticLevel.OK:
                system_health = DiagnosticLevel.WARN
        
        return SystemDiagnostics(
            system_name=system_name,
            timestamp=time.time(),
            component_states=self._component_states.copy(),
            component_diagnostics=self._component_diagnostics.copy(),
            diagnostic_propagation=dict(self._diagnostic_propagation),
            system_health=system_health,
            critical_errors=critical_errors
        )

