#!/usr/bin/env python3

"""
    System orchestration and lifecycle management.
    
    System is the top-level container that discovers components,
    builds the connection graph, and manages execution.
"""

# BAM
from lk.common.component import Component, LifecycleComponent, ComponentLifecycleMixin, LifecycleState
from lk.common.node import Node, NodeConfig
from lk.common.graph import ConnectionGraph, Connection
from lk.common.port import Port, InputPort, OutputPort

# PYTHON
from typing import List, Optional, Dict, Any
from dataclasses import dataclass, field
import time


@dataclass
class SystemConfig:
    """
        Base configuration for systems.
        
        Users can subclass this to add system-specific configuration.
    """
    name: str = "system"
    rate_hz: Optional[float] = None  # Target execution rate
    max_iterations: Optional[int] = None  # Stop after N iterations (None = infinite)
    
    # Debug options
    verbose: bool = False
    validate_graph: bool = True


class System:
    """
        Top-level system orchestrator.
        
        Discovers components and nodes, builds connection graph,
        manages lifecycle, and runs execution loop.
        
        Supports multiple API styles:
        1. Functional factory
        2. OOP with inheritance
        3. Builder pattern
    """
    
    def __init__(self, 
                 nodes: Optional[List[Node]] = None,
                 components: Optional[List[Component]] = None,
                 inputs: Optional[List[Port]] = None,
                 outputs: Optional[List[Port]] = None,
                 subsystems: Optional[List['System']] = None,
                 config: Optional[SystemConfig] = None):
        """
            Initialize system.
            
            Args:
                nodes: List of nodes to include
                components: List of components to include
                inputs: Public input ports
                outputs: Public output ports
                subsystems: Nested subsystems
                config: System configuration
        """
        self.config = config or SystemConfig()
        
        # Collections (will be discovered + manually added)
        self._nodes: List[Node] = nodes or []
        self._components: List[Component] = components or []
        self._subsystems: List[System] = subsystems or []
        
        # Public API ports
        self._inputs: List[Port] = inputs or []
        self._outputs: List[Port] = outputs or []
        
        # Connection graph
        self.graph = ConnectionGraph()
        
        # Runtime state
        self._configured = False
        self._active = False
        self._iteration = 0
        
        # Auto-discover components and nodes from attributes
        self._discover_components_and_nodes()
    
    def _discover_components_and_nodes(self):
        """
            Automatically discover components and nodes assigned as attributes.
            
            This enables the class-based API where users define components
            as self.agent, self.env, etc.
        """
        for attr_name in dir(self):
            if attr_name.startswith('_'):
                continue
            
            try:
                attr = getattr(self, attr_name)
                
                if isinstance(attr, Component):
                    if attr not in self._components:
                        self._components.append(attr)
                
                elif isinstance(attr, Node):
                    if attr not in self._nodes:
                        self._nodes.append(attr)
                
                elif isinstance(attr, System):
                    if attr not in self._subsystems:
                        self._subsystems.append(attr)
                        
            except AttributeError:
                continue
    
    def add_node(self, node: Node):
        """Add a node to the system."""
        if node not in self._nodes:
            self._nodes.append(node)
    
    def add_component(self, component: Component):
        """Add a component to the system."""
        if component not in self._components:
            self._components.append(component)
    
    def add_input(self, port: Port):
        """Add a public input port."""
        if port not in self._inputs:
            self._inputs.append(port)
    
    def add_output(self, port: Port):
        """Add a public output port."""
        if port not in self._outputs:
            self._outputs.append(port)
    
    def connect(self, source: OutputPort, target: InputPort) -> Connection:
        """
            Create a connection between two ports.
            
            Args:
                source: Output port (data source)
                target: Input port (data target)
                
            Returns:
                The created connection
        """
        return self.graph.connect(source, target)
    
    def _build_graph(self):
        """
            Build the connection graph from components.
            
            Discovers connections by examining port connections.
        """
        # Add all components to graph
        for component in self._components:
            # Add connections from component's ports
            for port in component.inputs.all_ports():
                for conn in port.connections:
                    self.graph.add_connection(conn)
            
            for port in component.outputs.all_ports():
                for conn in port.connections:
                    self.graph.add_connection(conn)
        
        # Validate graph if requested
        if self.config.validate_graph:
            is_valid, issues = self.graph.validate()
            if self.config.verbose:
                print(f"Graph validation: {'✓' if is_valid else '✗'}")
                for issue in issues:
                    print(f"  {issue}")
    
    def configure(self):
        """
            Configure all lifecycle components.
            
            Transition: UNCONFIGURED -> INACTIVE
        """
        if self._configured:
            return
        
        # Build connection graph
        self._build_graph()
        
        # Configure all lifecycle components
        for component in self._components:
            if isinstance(component, ComponentLifecycleMixin):
                if component.lifecycle_state == LifecycleState.UNCONFIGURED:
                    component.configure()
        
        # Configure subsystems
        for subsystem in self._subsystems:
            subsystem.configure()
        
        self._configured = True
        
        if self.config.verbose:
            print(f"System '{self.config.name}' configured")
            print(f"  Nodes: {len(self._nodes)}")
            print(f"  Components: {len(self._components)}")
            print(f"  Connections: {len(self.graph.connections)}")
    
    def activate(self):
        """
            Activate all lifecycle components.
            
            Components should call reset() here to populate initial outputs.
            Transition: INACTIVE -> ACTIVE
        """
        if not self._configured:
            self.configure()
        
        if self._active:
            return
        
        # Start nodes
        for node in self._nodes:
            node.start()
        
        # Activate all lifecycle components
        for component in self._components:
            if isinstance(component, ComponentLifecycleMixin):
                if component.lifecycle_state == LifecycleState.INACTIVE:
                    component.activate()
        
        # Activate subsystems
        for subsystem in self._subsystems:
            subsystem.activate()
        
        self._active = True
        
        if self.config.verbose:
            print(f"System '{self.config.name}' activated")
    
    def deactivate(self):
        """Deactivate all lifecycle components."""
        if not self._active:
            return
        
        # Deactivate subsystems
        for subsystem in self._subsystems:
            subsystem.deactivate()
        
        # Deactivate all lifecycle components
        for component in self._components:
            if isinstance(component, ComponentLifecycleMixin):
                if component.lifecycle_state == LifecycleState.ACTIVE:
                    component.deactivate()
        
        # Stop nodes
        for node in self._nodes:
            node.stop()
        
        self._active = False
        
        if self.config.verbose:
            print(f"System '{self.config.name}' deactivated")
    
    def shutdown(self):
        """Shutdown all lifecycle components."""
        if self._active:
            self.deactivate()
        
        # Shutdown subsystems
        for subsystem in self._subsystems:
            subsystem.shutdown()
        
        # Shutdown all lifecycle components
        for component in self._components:
            if isinstance(component, ComponentLifecycleMixin):
                if component.lifecycle_state == LifecycleState.INACTIVE:
                    component.shutdown()
        
        self._configured = False
        
        if self.config.verbose:
            print(f"System '{self.config.name}' shutdown")
    
    def step(self):
        """
            Execute one iteration of the system.
            
            Uses topological sort to execute components in dependency order.
        """
        if not self._active:
            raise RuntimeError("System must be activated before stepping")
        
        # Get execution order
        execution_order = self.graph.topological_sort()
        
        # Execute components in order
        for component in execution_order:
            # Call the component (triggers __call__ or forward)
            try:
                # Components handle their own execution logic
                # This is a placeholder - actual execution depends on component type
                pass
            except Exception as e:
                if self.config.verbose:
                    print(f"Error executing {component.name}: {e}")
        
        # Transfer data through connections
        self.graph.transfer_all()
        
        self._iteration += 1
    
    def run(self, iterations: Optional[int] = None):
        """
            Run the system for N iterations.
            
            Args:
                iterations: Number of iterations (None = use config, None in config = infinite)
        """
        if not self._active:
            self.activate()
        
        max_iter = iterations or self.config.max_iterations
        
        rate_limiter = None
        if self.config.rate_hz:
            dt = 1.0 / self.config.rate_hz
            rate_limiter = lambda: time.sleep(dt)
        
        try:
            if max_iter is None:
                # Run forever
                while True:
                    self.step()
                    if rate_limiter:
                        rate_limiter()
            else:
                # Run for N iterations
                for _ in range(max_iter):
                    self.step()
                    if rate_limiter:
                        rate_limiter()
        except KeyboardInterrupt:
            if self.config.verbose:
                print("\nSystem interrupted by user")
        finally:
            self.deactivate()
    
    def launch(self, iterations: Optional[int] = None):
        """
            Launch the system (configure, activate, run).
            
            Args:
                iterations: Number of iterations to run
        """
        try:
            self.configure()
            self.activate()
            self.run(iterations)
        except Exception as e:
            print(f"Error launching system: {e}")
            raise
        finally:
            self.shutdown()
    
    @property
    def nodes(self) -> List[Node]:
        """Get all nodes in the system."""
        return self._nodes.copy()
    
    @property
    def components(self) -> List[Component]:
        """Get all components in the system."""
        return self._components.copy()
    
    @property
    def inputs(self) -> List[Port]:
        """Get public input ports."""
        return self._inputs.copy()
    
    @property
    def outputs(self) -> List[Port]:
        """Get public output ports."""
        return self._outputs.copy()
    
    def plot_graph(self, 
                   to_file: Optional[str] = None,
                   show: bool = False,
                   rankdir: str = 'LR') -> Optional[str]:
        """
            Visualize the system graph using Mermaid.js.
            
            Similar to Keras's plot_model function.
            
            Args:
                to_file: Path to save HTML file (None = temporary file)
                show: Whether to open in browser
                rankdir: Layout direction ('LR' or 'TB')
                
            Returns:
                Path to generated HTML file
            
            Example:
                system.plot_graph(to_file='my_system.html', show=True)
        """
        from lk.utils.plot_graph import plot_system
        return plot_system(self, to_file=to_file, show=show, rankdir=rankdir)
    
    def to_mermaid(self, rankdir: str = 'LR') -> str:
        """
            Convert system to Mermaid diagram format.
            
            Args:
                rankdir: Layout direction ('LR' or 'TB')
                
            Returns:
                Mermaid diagram string
                
            Example:
                mermaid_str = system.to_mermaid()
                print(mermaid_str)
        """
        from lk.utils.plot_graph import system_to_mermaid
        return system_to_mermaid(self, rankdir=rankdir)
    
    def launch_viewer(self, port: int = 8050, debug: bool = False, live_update: bool = False, update_interval_ms: int = 1000):
        """
            Launch interactive Dash Cytoscape viewer.
            
            Opens a web-based interactive graph viewer with zoom, pan,
            and component inspection capabilities.
            
            Args:
                port: Port to run web server on
                debug: Enable debug mode
                live_update: Enable real-time updates of the graph
                update_interval_ms: Update interval in milliseconds (if live_update=True)
                
            Example:
                system.launch_viewer(port=8050)
                system.launch_viewer(port=8050, live_update=True, update_interval_ms=500)
        """
        from lk.utils.plot_graph import launch_interactive_viewer
        launch_interactive_viewer(self, port=port, debug=debug, live_update=live_update, update_interval_ms=update_interval_ms)
    
    def __repr__(self) -> str:
        return (
            f"System(name={self.config.name}, nodes={len(self._nodes)}, "
            f"components={len(self._components)}, iteration={self._iteration})"
        )

