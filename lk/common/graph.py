#!/usr/bin/env python3

"""
    Connection graph for managing component topology.
    
    Tracks connections between ports and validates the graph structure.
    Provides execution scheduling for sequential mode.
"""

# BAM
from lk.common.port import Port, InputPort, OutputPort
from lk.common.component import Component

# PYTHON
from typing import List, Set, Dict, Optional, Tuple
from dataclasses import dataclass


@dataclass
class Connection:
    """
        Represents a data flow connection between two ports.
        
        Connects an output port (source) to an input port (target).
    """
    source: OutputPort
    target: InputPort
    
    def __post_init__(self):
        """Validate connection and register with ports."""
        # Type compatibility check
        if self.source.msg_type != self.target.msg_type:
            # Allow subclass connections
            if not issubclass(self.source.msg_type, self.target.msg_type):
                raise TypeError(
                    f"Port type mismatch: {self.source.msg_type.__name__} -> "
                    f"{self.target.msg_type.__name__}"
                )
        
        # Register connection with ports
        self.source.connect(self)
        self.target.connect(self)
    
    def transfer(self):
        """Transfer data from source to target port."""
        self.target.value = self.source.value
    
    def disconnect(self):
        """Remove this connection from both ports."""
        self.source.disconnect(self)
        self.target.disconnect(self)
    
    def __repr__(self) -> str:
        src = f"{self.source.owner.name}.{self.source.name}" if self.source.owner else self.source.name
        tgt = f"{self.target.owner.name}.{self.target.name}" if self.target.owner else self.target.name
        return f"Connection({src} -> {tgt})"


class ConnectionGraph:
    """
        Manages the topology of component connections.
        
        Validates graph structure, detects cycles, and provides
        execution ordering for sequential mode.
    """
    
    def __init__(self):
        """Initialize empty connection graph."""
        self._connections: List[Connection] = []
        self._components: Set[Component] = set()
    
    def add_connection(self, connection: Connection):
        """
            Add a connection to the graph.
            
            Args:
                connection: Connection to add
        """
        if connection not in self._connections:
            self._connections.append(connection)
            
            # Track components
            if connection.source.owner:
                self._components.add(connection.source.owner)
            if connection.target.owner:
                self._components.add(connection.target.owner)
    
    def remove_connection(self, connection: Connection):
        """
            Remove a connection from the graph.
            
            Args:
                connection: Connection to remove
        """
        if connection in self._connections:
            self._connections.remove(connection)
            connection.disconnect()
    
    def connect(self, source: OutputPort, target: InputPort) -> Connection:
        """
            Create and add a connection between two ports.
            
            Args:
                source: Output port (data source)
                target: Input port (data target)
                
            Returns:
                The created connection
        """
        connection = Connection(source=source, target=target)
        self.add_connection(connection)
        return connection
    
    @property
    def connections(self) -> List[Connection]:
        """Get all connections in the graph."""
        return self._connections.copy()
    
    @property
    def components(self) -> List[Component]:
        """Get all components in the graph."""
        return list(self._components)
    
    def get_component_dependencies(self) -> Dict[Component, Set[Component]]:
        """
            Build component dependency graph.
            
            Returns:
                Dict mapping each component to its dependencies
                (components it receives data from)
        """
        deps: Dict[Component, Set[Component]] = {c: set() for c in self._components}
        
        for conn in self._connections:
            if conn.source.owner and conn.target.owner:
                # target depends on source
                deps[conn.target.owner].add(conn.source.owner)
        
        return deps
    
    def detect_cycles(self) -> List[List[Component]]:
        """
            Detect cycles in the component graph.
            
            Returns:
                List of cycles (each cycle is a list of components)
        """
        deps = self.get_component_dependencies()
        cycles = []
        
        def dfs(node: Component, path: List[Component], visited: Set[Component]):
            """Depth-first search to find cycles."""
            if node in path:
                # Found cycle
                cycle_start = path.index(node)
                cycle = path[cycle_start:]
                if cycle not in cycles:
                    cycles.append(cycle)
                return
            
            if node in visited:
                return
            
            visited.add(node)
            path.append(node)
            
            for dep in deps.get(node, set()):
                dfs(dep, path.copy(), visited)
        
        visited: Set[Component] = set()
        for component in self._components:
            if component not in visited:
                dfs(component, [], visited)
        
        return cycles
    
    def topological_sort(self) -> List[Component]:
        """
            Compute topological ordering of components.
            
            Returns components in execution order (dependencies first).
            Works for DAGs; for cyclic graphs, breaks cycles arbitrarily.
            
            Returns:
                List of components in execution order
        """
        deps = self.get_component_dependencies()
        in_degree = {c: len(deps[c]) for c in self._components}
        
        # Find all nodes with no dependencies
        queue = [c for c in self._components if in_degree[c] == 0]
        result = []
        
        while queue:
            # Sort by name for deterministic ordering
            queue.sort(key=lambda c: c.name)
            node = queue.pop(0)
            result.append(node)
            
            # Reduce in-degree for dependent nodes
            for component in self._components:
                if node in deps[component]:
                    in_degree[component] -= 1
                    if in_degree[component] == 0:
                        queue.append(component)
        
        # If not all components are in result, there's a cycle
        # Add remaining components in arbitrary order
        remaining = [c for c in self._components if c not in result]
        if remaining:
            remaining.sort(key=lambda c: c.name)
            result.extend(remaining)
        
        return result
    
    def validate(self) -> Tuple[bool, List[str]]:
        """
            Validate the connection graph.
            
            Checks for:
            - Type compatibility
            - Disconnected components
            - Cycles (warning, not error)
            
            Returns:
                Tuple of (is_valid, list of warning/error messages)
        """
        issues = []
        
        # Check for cycles
        cycles = self.detect_cycles()
        if cycles:
            issues.append(f"Warning: Found {len(cycles)} cycle(s) in graph")
            for i, cycle in enumerate(cycles):
                cycle_names = " -> ".join(c.name for c in cycle)
                issues.append(f"  Cycle {i+1}: {cycle_names}")
        
        # Check for disconnected components
        for component in self._components:
            has_input = any(
                conn.target.owner == component for conn in self._connections
            )
            has_output = any(
                conn.source.owner == component for conn in self._connections
            )
            
            if not has_input and not has_output:
                issues.append(f"Warning: Component {component.name} is disconnected")
        
        # Type checks already done in Connection.__post_init__
        
        is_valid = not any("Error" in issue for issue in issues)
        return is_valid, issues
    
    def transfer_all(self):
        """Transfer data through all connections."""
        for conn in self._connections:
            conn.transfer()
    
    def __repr__(self) -> str:
        return f"ConnectionGraph(components={len(self._components)}, connections={len(self._connections)})"

