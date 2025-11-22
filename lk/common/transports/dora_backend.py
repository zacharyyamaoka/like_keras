#!/usr/bin/env python3

"""
    Dora RS Backend Compiler
    
    Compiles like-keras System to Dora dataflow format.
    
    Dora dataflow structure:
    ```yaml
    nodes:
      - id: node_name
        operator:
          python: path/to/script.py
        inputs:
          input_name:
            source: other_node/output_name
        outputs:
          - output_name
    ```
    
    References:
    - https://dora-rs.ai/docs/guides/dataflow
    - https://github.com/dora-rs/dora
"""

# BAM
from lk.common.backend import Backend, BackendConfig
from lk.common.system import System
from lk.common.node import Node
from lk.common.component import Component
from lk.common.graph import Connection

# PYTHON
from typing import Any
from pathlib import Path
import yaml
import subprocess
import shutil


class DoraBackend(Backend):
    """
        Dora RS backend compiler.
        
        Converts like-keras System to Dora dataflow.yml format,
        then uses Dora's native tools to launch.
    """
    
    def compile(self, system: System) -> Path:
        """
            Compile system to Dora dataflow.yml.
            
            Args:
                system: System to compile
                
            Returns:
                Path to generated dataflow.yml
        """
        if self.config.verbose:
            print(f"[DoraBackend] Compiling system '{system.config.name}' to Dora dataflow...")
        
        # Build dataflow structure
        dataflow = self._build_dataflow(system)
        
        # Determine output path
        if self.config.output_dir:
            output_dir = Path(self.config.output_dir)
            output_dir.mkdir(parents=True, exist_ok=True)
        else:
            output_dir = Path.cwd() / "build" / system.config.name
            output_dir.mkdir(parents=True, exist_ok=True)
        
        if self.config.output_file:
            output_path = output_dir / self.config.output_file
        else:
            output_path = output_dir / "dataflow.yml"
        
        # Write YAML
        with open(output_path, 'w') as f:
            yaml.dump(dataflow, f, default_flow_style=False, sort_keys=False)
        
        if self.config.verbose:
            print(f"[DoraBackend] ✓ Compiled to: {output_path}")
            print(f"[DoraBackend]   Nodes: {len(dataflow.get('nodes', []))}")
        
        return output_path
    
    def _build_dataflow(self, system: System) -> dict[str, Any]:
        """
            Build Dora dataflow dictionary from system.
            
            Args:
                system: System to convert
                
            Returns:
                Dataflow dictionary (to be serialized to YAML)
        """
        dataflow: dict[str, Any] = {}
        
        # Nodes
        nodes_list = []
        
        for node in system.nodes:
            node_dict = self._build_node(node, system)
            if node_dict:
                nodes_list.append(node_dict)
        
        dataflow['nodes'] = nodes_list
        
        return dataflow
    
    def _build_node(self, node: Node, system: System) -> dict[str, Any]:
        """
            Build Dora node configuration.
            
            Args:
                node: Node to convert
                system: Parent system (for finding connections)
                
            Returns:
                Node dictionary for dataflow
        """
        node_dict: dict[str, Any] = {
            'id': node.name,
        }
        
        # Determine operator type
        # For now, assume Python nodes
        # TODO: Support other languages via node metadata
        operator = self._build_operator(node)
        if operator:
            node_dict['operator'] = operator
        
        # Build inputs (from connections in graph)
        inputs = self._build_node_inputs(node, system)
        if inputs:
            node_dict['inputs'] = inputs
        
        # Build outputs (from component output ports)
        outputs = self._build_node_outputs(node)
        if outputs:
            node_dict['outputs'] = outputs
        
        return node_dict
    
    def _build_operator(self, node: Node) -> dict[str, Any]:
        """
            Build operator specification for a node.
            
            Args:
                node: Node to build operator for
                
            Returns:
                Operator dictionary
        """
        # Check if node has operator metadata
        if hasattr(node, 'dora_operator'):
            return node.dora_operator
        
        # Default: Python operator
        # Generate a Python script path (would need to be implemented)
        script_path = f"nodes/{node.name}.py"
        
        return {
            'python': script_path
        }
    
    def _build_node_inputs(self, node: Node, system: System) -> dict[str, Any]:
        """
            Build input specifications for a node.
            
            Looks at connections in the graph to find inputs.
            
            Args:
                node: Node to build inputs for
                system: Parent system
                
            Returns:
                Inputs dictionary
        """
        inputs = {}
        
        # Find all connections where this node is the target
        for conn in system.graph.connections:
            if conn.target.owner in node.components:
                # This connection targets a component in this node
                input_name = conn.target.name
                source_node_name = self._get_node_name_for_component(
                    conn.source.owner, system
                )
                source_port_name = conn.source.name
                
                inputs[input_name] = {
                    'source': f"{source_node_name}/{source_port_name}"
                }
        
        return inputs
    
    def _build_node_outputs(self, node: Node) -> list[str]:
        """
            Build output specifications for a node.
            
            Args:
                node: Node to build outputs for
                
            Returns:
                List of output port names
        """
        outputs = []
        
        # Collect all output ports from components in this node
        for component in node.components:
            for port in component.outputs.all_ports():
                outputs.append(port.name)
        
        return outputs
    
    def _get_node_name_for_component(self, component: Component, system: System) -> str:
        """
            Find which node a component belongs to.
            
            Args:
                component: Component to find
                system: System to search
                
            Returns:
                Node name
        """
        for node in system.nodes:
            if component in node.components:
                return node.name
        return "unknown"
    
    def launch(self, compiled_path: Path) -> subprocess.Popen:
        """
            Launch compiled dataflow using Dora CLI.
            
            Args:
                compiled_path: Path to dataflow.yml
                
            Returns:
                Process handle for dora daemon/coordinator
        """
        # Check if dora is installed
        if not shutil.which('dora'):
            raise RuntimeError(
                "Dora RS not found. Please install: https://dora-rs.ai/docs/guides/installation"
            )
        
        if self.config.verbose:
            print(f"[DoraBackend] Launching with Dora: {compiled_path}")
        
        # Start dora coordinator if not running
        # (In practice, might want to check if already running)
        try:
            subprocess.run(['dora', 'up'], check=True, capture_output=True)
            if self.config.verbose:
                print(f"[DoraBackend] Dora coordinator started")
        except subprocess.CalledProcessError:
            # Coordinator might already be running
            pass
        
        # Launch dataflow
        process = subprocess.Popen(
            ['dora', 'start', str(compiled_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        
        if self.config.verbose:
            print(f"[DoraBackend] ✓ Dataflow launched (PID: {process.pid})")
        
        return process
    
    def shutdown(self, process: subprocess.Popen):
        """
            Shutdown Dora dataflow.
            
            Args:
                process: Process handle from launch()
        """
        if self.config.verbose:
            print(f"[DoraBackend] Shutting down dataflow...")
        
        # Stop the dataflow
        subprocess.run(['dora', 'stop'], capture_output=True)
        
        # Terminate the process
        if process:
            process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
        
        if self.config.verbose:
            print(f"[DoraBackend] ✓ Shutdown complete")


if __name__ == "__main__":
    # Example usage
    from lk.common.system import System
    from lk.common.node import Node
    from lk.agent import Agent
    from lk.env import Env
    
    # Create a simple system
    node = Node(name="main_loop")
    agent = Agent.from_node(node)
    env = Env.from_node(node)
    
    system = System(
        nodes=[node],
        components=[agent, env],
        config=System.Config(name="test_system"),
    )
    
    # Compile to Dora dataflow
    backend = DoraBackend(BackendConfig(
        backend_type=BackendType.DORA_RS,
        output_dir=Path("./build"),
        verbose=True,
    ))
    
    dataflow_path = backend.compile(system)
    print(f"\nGenerated dataflow at: {dataflow_path}")
    
    # Print the generated YAML
    print("\nGenerated dataflow.yml:")
    print("=" * 70)
    with open(dataflow_path) as f:
        print(f.read())

