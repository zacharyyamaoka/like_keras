#!/usr/bin/env python3

"""
System orchestration and lifecycle management.

System is the top-level container that discovers components,
builds the connection graph, and manages execution.
"""

# BAM
import time
from dataclasses import dataclass

# PYTHON
from typing import Any, Optional

from lk.common.component import (
    Component,
    ComponentLifecycleMixin,
    LifecycleState,
)
from lk.common.graph import Connection, ConnectionGraph
from lk.common.node import Node
from lk.common.port import InputPort, OutputPort, Port


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

    @dataclass
    class Config:
        """
        Base configuration for systems.

        Users can subclass this to add system-specific configuration.
        """

        name: str = "system"
        rate_hz: float | None = None  # Target execution rate
        max_iterations: int | None = None  # Stop after N iterations (None = infinite)

        # Debug options
        verbose: bool = False
        validate_graph: bool = True

    def __init__(
        self,
        nodes: list[Node] | None = None,
        components: list[Component] | None = None,
        inputs: list[Port] | None = None,
        outputs: list[Port] | None = None,
        subsystems: list["System"] | None = None,
        config: Optional["System.Config"] = None,
    ):
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
        self.config = config or System.Config()

        # Collections (will be discovered + manually added)
        self._nodes: list[Node] = nodes or []
        self._components: list[Component] = components or []
        self._subsystems: list[System] = subsystems or []

        # Public API ports
        self._inputs: list[Port] = inputs or []
        self._outputs: list[Port] = outputs or []

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
        
        Also auto-discovers nodes from components' node assignments.
        """
        for attr_name in dir(self):
            if attr_name.startswith("_"):
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
        
        # Auto-discover nodes from component assignments
        self._discover_nodes_from_components()
    
    def _discover_nodes_from_components(self):
        """
        Auto-discover nodes from components' node assignments.
        
        Collects unique nodes that components are assigned to.
        """
        discovered_nodes = {}
        
        # Collect from components
        for component in self._components:
            if component.node is not None:
                if component.node.name not in discovered_nodes:
                    discovered_nodes[component.node.name] = component.node
        
        # Collect from subsystems
        for subsystem in self._subsystems:
            for node in subsystem.nodes:
                if node.name not in discovered_nodes:
                    discovered_nodes[node.name] = node
        
        # Add discovered nodes to our list if not already present
        for node in discovered_nodes.values():
            if node not in self._nodes:
                self._nodes.append(node)
    
    def get_node(self, name: str) -> Optional[Node]:
        """
        Get a node by name.
        
        Args:
            name: Node name
            
        Returns:
            Node if found, None otherwise
        """
        for node in self._nodes:
            if node.name == name:
                return node
        return None

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

    def add_web_server(
        self, web_server: Any | None = None, auto_connect: bool = True
    ) -> Any:
        """
        Add web server component for monitoring.

        Args:
            web_server: WebServerComponent instance (creates one if None)
            auto_connect: Auto-connect to all state/diagnostics ports

        Returns:
            The web server component
        """
        from lk.common.web_server import WebServerComponent, WebServerConfig

        if web_server is None:
            config = WebServerConfig(auto_connect_all=auto_connect)
            web_server = WebServerComponent(config=config)

        self.add_component(web_server)
        return web_server

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

    def run(self, iterations: int | None = None):
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

<<<<<<< Current (Your changes)
    def launch(self, iterations: int | None = None, transport: str | None = None):
        """
        Launch the system (configure, activate, run).

        Can launch using different transports:
        - None/"native": Run in same process (default)
        - "dora": Compile to Dora dataflow and launch with Dora transport
        - "ros2": Compile to ROS2 launch file and use ROS2/DDS transport

        Args:
            iterations: Number of iterations to run
            transport: Transport to use ("native", "dora", "ros2", or None for native)
        """
        if transport and transport != "native":
            # Use transport compilation
            return self._launch_with_transport(transport)

        # Native launch (same process)
        try:
            self.configure()
            self.activate()
            self.run(iterations)
        except Exception as e:
            print(f"Error launching system: {e}")
            raise
        finally:
            self.shutdown()
=======
    def launch(self, iterations: Optional[int] = None, engine: Optional[str] = None):
        """
        Launch the system (compile if needed, then execute).
        
        This method will:
        1. Determine the execution engine from nodes
        2. Get the engine instance
        3. Compile the system for that engine (if applicable)
        4. Execute the system
        
        Args:
            iterations: Number of iterations to run
            engine: Force specific engine (overrides node configs)
        """
        from lk.engines import get_engine
        
        # Determine which engine to use
        engine_name = engine or self._determine_engine()
        
        # Get engine instance
        engine_instance = get_engine(engine_name)
        
        # Compile system
        config_path = engine_instance.compile(self)
        
        # Launch via engine
        engine_instance.launch(self, config_path, iterations=iterations)
    
    def _determine_engine(self) -> str:
        """
        Determine which execution engine to use.
        
        Examines node configurations to determine engine.
        All nodes must use the same engine (heterogeneous not supported yet).
        
        Returns:
            Engine name ('native', 'dora', 'ros2', etc.)
        """
        if not self._nodes:
            # No explicit nodes, use native
            return 'native'
        
        # Collect unique engines
        engines = set()
        for node in self._nodes:
            engines.add(node.engine)
        
        if len(engines) == 0:
            return 'native'
        elif len(engines) == 1:
            return list(engines)[0]
        else:
            raise ValueError(
                f"Heterogeneous engines not supported yet. "
                f"Found engines: {engines}. All nodes must use the same engine."
            )
    
    def get_components_by_node(self) -> Dict[str, List[Component]]:
        """
        Group components by their assigned nodes.
        
        Returns:
            Dictionary mapping node names to lists of components
        """
        node_components = {}
        
        for component in self._components:
            if component.node is None:
                # Component not assigned to node, use default
                node_name = 'default'
            else:
                node_name = component.node.name
            
            if node_name not in node_components:
                node_components[node_name] = []
            node_components[node_name].append(component)
        
        return node_components
>>>>>>> Incoming (Background Agent changes)

    def _launch_with_transport(self, transport_name: str):
        """
        Launch system using a transport (Dora, ROS2, etc.).

        Args:
            transport_name: Name of transport to use
        """
        from lk.common.transport import TransportConfig, TransportType, create_transport

        # Map string names to TransportType
        transport_map = {
            "dora": TransportType.DORA_RS,
            "dora_rs": TransportType.DORA_RS,
            "ros2": TransportType.ROS2,
        }

        if transport_name not in transport_map:
            raise ValueError(
                f"Unknown transport '{transport_name}'. "
                f"Available: {list(transport_map.keys())}"
            )

        transport_type = transport_map[transport_name]

        # Create transport config
        config = TransportConfig(
            transport_type=transport_type,
            verbose=self.config.verbose,
        )

        # Create transport and compile+launch
        transport = create_transport(transport_type, config)

        if self.config.verbose:
            print(f"Using {transport_name} transport for message passing")

        # Compile and launch
        process = transport.compile_and_launch(self)

        if process:
            try:
                # Wait for process to complete (or Ctrl+C)
                process.wait()
            except KeyboardInterrupt:
                if self.config.verbose:
                    print("\nShutting down...")
                transport.shutdown(process)

    def compile_to_transport(
        self, transport: str, output_path: str | None = None
    ) -> Path:
        """
        Compile system to transport format without launching.

        Useful for:
        - Generating Dora dataflow.yml for inspection
        - Generating ROS2 launch.py for customization
        - CI/CD pipelines

        Args:
            transport: Transport to compile to ("dora", "ros2")
            output_path: Where to save compiled file

        Returns:
            Path to compiled file

        Example:
            system.compile_to_transport("dora", "my_dataflow.yml")
        """
        from lk.common.transport import TransportConfig, TransportType, create_transport

        transport_map = {
            "dora": TransportType.DORA_RS,
            "ros2": TransportType.ROS2,
        }

        if transport not in transport_map:
            raise ValueError(f"Unknown transport '{transport}'")

        transport_type = transport_map[transport]
        config = TransportConfig(
            transport_type=transport_type,
            output_file=output_path,
            verbose=self.config.verbose,
            auto_launch=False,
        )

        transport_instance = create_transport(transport_type, config)
        return transport_instance.compile(self)

    @property
    def nodes(self) -> list[Node]:
        """Get all nodes in the system."""
        return self._nodes.copy()

    @property
    def components(self) -> list[Component]:
        """Get all components in the system."""
        return self._components.copy()

    @property
    def inputs(self) -> list[Port]:
        """Get public input ports."""
        return self._inputs.copy()

    @property
    def outputs(self) -> list[Port]:
        """Get public output ports."""
        return self._outputs.copy()

    def plot_graph(
        self, to_file: str | None = None, show: bool = False, rankdir: str = "LR"
    ) -> str | None:
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

    def to_mermaid(self, rankdir: str = "LR") -> str:
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

    def launch_viewer(
        self,
        port: int = 8050,
        debug: bool = False,
        live_update: bool = False,
        update_interval_ms: int = 1000,
    ):
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

        launch_interactive_viewer(
            self,
            port=port,
            debug=debug,
            live_update=live_update,
            update_interval_ms=update_interval_ms,
        )

    def __repr__(self) -> str:
        return (
            f"System(name={self.config.name}, nodes={len(self._nodes)}, "
            f"components={len(self._components)}, iteration={self._iteration})"
        )


# Backwards compatibility alias
SystemConfig = System.Config
