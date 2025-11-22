#!/usr/bin/env python3

"""
    Multi-Engine Example
    
    Demonstrates running the same system on different engines:
    - Native: Fast development iteration
    - Dora: Production deployment
    
    Shows how to:
    - Define atomic components
    - Assign nodes for deployment
    - Switch between engines
"""

# BAM
from lk.common.component import Component
from lk.common.node import Node
from lk.common.system import System

# PYTHON
import numpy as np
from typing import Optional
from dataclasses import dataclass


# =============================================================================
# Simple Components
# =============================================================================

class Generator(Component):
    """Generates random data."""
    
    @dataclass
    class Config:
        """Generator configuration."""
        size: int = 10
        seed: Optional[int] = None
    
    def __init__(self, node: Optional[Node] = None, config: Optional[Config] = None):
        super().__init__('generator', node=node, config=config)
        
        # Normal Python internals
        if self.config.seed is not None:
            np.random.seed(self.config.seed)
    
    def forward(self):
        """Generate random data."""
        data = np.random.randn(self.config.size)
        if hasattr(self, '_iteration'):
            self._iteration += 1
        else:
            self._iteration = 0
        
        print(f"  Generator: Generated data (iteration {self._iteration})")
        return data


class Processor(Component):
    """Processes data (squares it)."""
    
    def __init__(self, node: Optional[Node] = None):
        super().__init__('processor', node=node)
    
    def forward(self, data):
        """Process data."""
        result = data ** 2
        print(f"  Processor: Processed data, sum={result.sum():.2f}")
        return result


class Analyzer(Component):
    """Analyzes processed data."""
    
    def __init__(self, node: Optional[Node] = None):
        super().__init__('analyzer', node=node)
        self.total = 0.0
    
    def forward(self, data):
        """Analyze data."""
        mean = data.mean()
        self.total += mean
        print(f"  Analyzer: Mean={mean:.2f}, Total={self.total:.2f}")
        return mean


# =============================================================================
# Example 1: Native Engine (Development)
# =============================================================================

def example_native():
    """Run on native engine for development."""
    print("\n" + "=" * 70)
    print("Example 1: Native Engine (Development)")
    print("=" * 70)
    
    # Create components (no explicit nodes = native)
    generator = Generator(config=Generator.Config(size=5, seed=42))
    processor = Processor()
    analyzer = Analyzer()
    
    # Create system
    system = System(
        components=[generator, processor, analyzer],
        config=System.Config(
            name='pipeline_native',
            verbose=True,
            max_iterations=3
        )
    )
    
    print("\nLaunching with native engine...")
    system.launch(engine='native')
    
    print("\n✅ Native execution completed")


# =============================================================================
# Example 2: Dora Engine (Production - Compilation Only)
# =============================================================================

def example_dora_compile():
    """Compile to Dora engine (doesn't launch - requires dora-rs installed)."""
    print("\n" + "=" * 70)
    print("Example 2: Dora Engine (Compilation)")
    print("=" * 70)
    
    # Define deployment nodes
    node1 = Node('worker1', machine='local', engine='dora')
    node2 = Node('worker2', machine='local', engine='dora')
    
    # Create components with node assignment
    generator = Generator(node=node1, config=Generator.Config(size=5, seed=42))
    processor = Processor(node=node1)  # Same node as generator
    analyzer = Analyzer(node=node2)     # Different node
    
    # Create system
    system = System(
        components=[generator, processor, analyzer],
        config=System.Config(
            name='pipeline_dora',
            verbose=True
        )
    )
    
    print("\nNodes discovered from components:")
    for node in system.nodes:
        print(f"  - {node.name} (machine={node.machine}, engine={node.engine})")
    
    print("\nComponents by node:")
    for node_name, components in system.get_components_by_node().items():
        comp_names = [c.name for c in components]
        print(f"  - {node_name}: {comp_names}")
    
    try:
        print("\nCompiling to Dora dataflow...")
        config_path = system.compile(engine='dora')
        print(f"✅ Generated Dora config: {config_path}")
        
        print("\nGenerated dataflow.yaml:")
        with open(config_path, 'r') as f:
            print(f.read())
        
        print("\n📝 To run with Dora:")
        print(f"   dora start {config_path}")
        
    except Exception as e:
        print(f"⚠️  Compilation failed: {e}")
        print("   (This is expected if dora-rs is not installed)")


# =============================================================================
# Example 3: Distributed Dora (Multi-Machine)
# =============================================================================

def example_dora_distributed():
    """Example of distributed Dora deployment."""
    print("\n" + "=" * 70)
    print("Example 3: Dora Engine (Distributed)")
    print("=" * 70)
    
    # Define nodes on different machines
    jetson = Node('jetson', machine='192.168.1.10', engine='dora')
    nuc = Node('nuc', machine='192.168.1.11', engine='dora')
    
    # Assign components to specific machines
    generator = Generator(node=jetson, config=Generator.Config(size=5))
    processor = Processor(node=jetson)  # Heavy computation on Jetson
    analyzer = Analyzer(node=nuc)        # Analysis on NUC
    
    # Create system
    system = System(
        components=[generator, processor, analyzer],
        config=System.Config(
            name='pipeline_distributed',
            verbose=True
        )
    )
    
    print("\nDistributed deployment:")
    for node in system.nodes:
        comps = [c.name for c in system.get_components_by_node()[node.name]]
        print(f"  - Machine {node.machine} ({node.name}): {comps}")
    
    try:
        print("\nCompiling distributed dataflow...")
        config_path = system.compile(engine='dora')
        print(f"✅ Generated config: {config_path}")
        
        print("\nGenerated dataflow.yaml:")
        with open(config_path, 'r') as f:
            print(f.read())
        
        print("\n📝 To run distributed:")
        print("   1. Start coordinator:")
        print("      dora coordinator")
        print("   2. Start daemons on each machine:")
        print("      # On Jetson (192.168.1.10):")
        print("      dora daemon --machine-id jetson --coordinator-addr <coordinator-ip>")
        print("      # On NUC (192.168.1.11):")
        print("      dora daemon --machine-id nuc --coordinator-addr <coordinator-ip>")
        print("   3. Launch dataflow:")
        print(f"      dora start {config_path} --coordinator-addr <coordinator-ip>")
        
    except Exception as e:
        print(f"⚠️  Compilation failed: {e}")


# =============================================================================
# Main
# =============================================================================

if __name__ == "__main__":
    print("like_keras Multi-Engine Example")
    print("Demonstrating execution on different engines")
    
    # Example 1: Native engine (actually runs)
    example_native()
    
    # Example 2: Dora compilation (generates config)
    example_dora_compile()
    
    # Example 3: Distributed Dora (generates distributed config)
    example_dora_distributed()
    
    print("\n" + "=" * 70)
    print("All examples completed!")
    print("=" * 70)
    print("\nKey takeaways:")
    print("  - Same components work on different engines")
    print("  - Native engine: fast development iteration")
    print("  - Dora engine: production deployment")
    print("  - Node assignment controls deployment")
    print("  - System auto-discovers nodes from components")

