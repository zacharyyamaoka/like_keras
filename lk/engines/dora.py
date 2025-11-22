#!/usr/bin/env python3

"""
    Dora-rs execution engine.
    
    Compiles like_keras Systems to Dora dataflow YAML files and
    launches them using the Dora CLI (dora start).
    
    Leverages Dora's coordinator/daemon architecture for distributed execution.
"""

# BAM
from lk.engines.engine import Engine

# PYTHON
from pathlib import Path
from typing import Optional, TYPE_CHECKING, Dict, List
import subprocess
import yaml

if TYPE_CHECKING:
    from lk.common.system import System
    from lk.common.component import Component
    from lk.common.node import Node


class DoraEngine(Engine):
    """
    Dora-rs execution engine.
    
    Compiles System graphs to Dora dataflow YAML and launches via Dora CLI.
    
    Features:
    - Multi-machine deployment (via Dora coordinator/daemon)
    - Zero-copy shared memory (local nodes)
    - Zenoh networking (distributed nodes)
    - Multi-language support (Python, Rust, C++, C)
    
    Use for:
    - Production deployments
    - Distributed robotics systems
    - High-performance applications
    """
    
    def __init__(self, build_dir: str = ".lk_build/dora"):
        """
        Initialize Dora engine.
        
        Args:
            build_dir: Directory for generated files
        """
        self.build_dir = Path(build_dir)
    
    def compile(self, system: "System") -> Optional[Path]:
        """
        Compile system to Dora dataflow YAML.
        
        Args:
            system: System to compile
            
        Returns:
            Path to generated dataflow.yaml
        """
        # Validate system
        is_valid, issues = self.validate(system)
        if not is_valid:
            raise ValueError(f"System validation failed:\n" + "\n".join(issues))
        
        # Create build directory
        self.build_dir.mkdir(parents=True, exist_ok=True)
        
        # Generate dataflow YAML
        dataflow = self._generate_dataflow(system)
        
        # Write to file
        dataflow_path = self.build_dir / "dataflow.yaml"
        with open(dataflow_path, 'w') as f:
            yaml.dump(dataflow, f, default_flow_style=False, sort_keys=False)
        
        if system.config.verbose:
            print(f"Generated Dora dataflow: {dataflow_path}")
        
        return dataflow_path
    
    def launch(self, system: "System", config_path: Optional[Path] = None, **kwargs):
        """
        Launch system using Dora CLI.
        
        Args:
            system: System to launch
            config_path: Path to dataflow.yaml
            **kwargs: Additional launch options
                - coordinator_addr: Address of Dora coordinator (for distributed)
                - detach: Run in background
        """
        if config_path is None:
            raise ValueError("config_path required for Dora engine")
        
        # Build dora command
        cmd = ['dora', 'start', str(config_path)]
        
        # Add coordinator address if provided (distributed mode)
        coordinator_addr = kwargs.get('coordinator_addr')
        if coordinator_addr:
            cmd.extend(['--coordinator-addr', coordinator_addr])
        
        # Detach flag
        if kwargs.get('detach', False):
            cmd.append('--detach')
        
        if system.config.verbose:
            print(f"Launching Dora dataflow: {' '.join(cmd)}")
        
        # Execute Dora CLI
        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Dora launch failed: {e}")
        except FileNotFoundError:
            raise RuntimeError(
                "Dora CLI not found. Please install dora-rs: "
                "pip install dora-rs-cli"
            )
    
    def validate(self, system: "System") -> tuple[bool, list[str]]:
        """
        Validate system for Dora execution.
        
        Args:
            system: System to validate
            
        Returns:
            Tuple of (is_valid, list_of_issues)
        """
        issues = []
        
        # Check that all nodes use dora engine
        for node in system.nodes:
            if node.engine != 'dora':
                issues.append(
                    f"Node '{node.name}' uses engine '{node.engine}'. "
                    "All nodes must use engine='dora' for Dora execution."
                )
        
        # Check for components without nodes
        for component in system.components:
            if component.node is None:
                issues.append(
                    f"Component '{component.name}' not assigned to a node. "
                    "All components must be assigned to nodes for Dora execution."
                )
        
        is_valid = len(issues) == 0
        return is_valid, issues
    
    def _generate_dataflow(self, system: "System") -> dict:
        """
        Generate Dora dataflow configuration.
        
        Args:
            system: System to convert
            
        Returns:
            Dataflow dict (to be serialized as YAML)
        """
        dataflow = {
            'nodes': []
        }
        
        # Group components by node
        node_components = system.get_components_by_node()
        
        # Generate a Dora node for each Node
        for node in system.nodes:
            node_name = node.name
            components = node_components.get(node_name, [])
            
            if not components:
                continue
            
            # Build Dora node configuration
            dora_node = self._generate_dora_node(node, components, system)
            dataflow['nodes'].append(dora_node)
        
        return dataflow
    
    def _generate_dora_node(
        self, 
        node: "Node", 
        components: List["Component"],
        system: "System"
    ) -> dict:
        """
        Generate Dora node configuration for a Node.
        
        Args:
            node: Node configuration
            components: Components assigned to this node
            system: Parent system
            
        Returns:
            Dora node dict
        """
        # Collect all inputs and outputs from components
        inputs = {}
        outputs = []
        
        for component in components:
            # Add component outputs
            for port_name, port in component.outputs.items():
                outputs.append(f"{component.name}/{port_name}")
            
            # Add component inputs (with connections)
            for port_name, port in component.inputs.items():
                if port.connections:
                    # Get source from connection
                    for conn in port.connections:
                        source = conn.source
                        source_id = f"{source.owner.name}/{source.name}"
                        inputs[f"{component.name}/{port_name}"] = source_id
        
        # Build node dict
        dora_node = {
            'id': node.name,
            'path': 'python',
            'args': [
                '-m', 'lk.engines.backends.dora_node',
                node.name,  # Node ID as argument
            ],
        }
        
        # Add inputs if any
        if inputs:
            dora_node['inputs'] = inputs
        
        # Add outputs if any
        if outputs:
            dora_node['outputs'] = outputs
        
        # Add deployment config if not local
        if node.machine != 'local':
            dora_node['_unstable_deploy'] = {
                'machine': node.machine
            }
        
        return dora_node

