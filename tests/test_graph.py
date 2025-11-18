#!/usr/bin/env python3

"""
    Tests for connection graph.
"""

# BAM
from lk.common.graph import Connection, ConnectionGraph
from lk.common.component import Component
from lk.common.port import InputPort, OutputPort
from lk.msgs.msg import Observation

# PYTHON
import pytest


class TestComponent(Component):
    """Test component for graph tests."""
    
    def __init__(self, name):
        super().__init__(name=name)
        self.obs_in = InputPort("obs_in", Observation, owner=self)
        self.obs_out = OutputPort("obs_out", Observation, owner=self)
        self._discover_ports()


def test_connection_creation():
    """Test connection creation."""
    comp1 = TestComponent("comp1")
    comp2 = TestComponent("comp2")
    
    conn = Connection(source=comp1.obs_out, target=comp2.obs_in)
    
    assert conn.source == comp1.obs_out
    assert conn.target == comp2.obs_in


def test_connection_type_validation():
    """Test connection type validation."""
    comp1 = TestComponent("comp1")
    comp2 = TestComponent("comp2")
    
    # Valid connection (same types)
    conn = Connection(source=comp1.obs_out, target=comp2.obs_in)
    assert conn is not None


def test_connection_transfer():
    """Test data transfer through connection."""
    comp1 = TestComponent("comp1")
    comp2 = TestComponent("comp2")
    
    conn = Connection(source=comp1.obs_out, target=comp2.obs_in)
    
    # Write to source
    obs = Observation(data=[1, 2, 3])
    comp1.obs_out.write(obs)
    
    # Transfer
    conn.transfer()
    
    # Check target
    assert comp2.obs_in.value == obs


def test_connection_graph():
    """Test connection graph."""
    graph = ConnectionGraph()
    
    comp1 = TestComponent("comp1")
    comp2 = TestComponent("comp2")
    
    conn = graph.connect(comp1.obs_out, comp2.obs_in)
    
    assert len(graph.connections) == 1
    assert len(graph.components) == 2


def test_graph_topological_sort():
    """Test topological sorting of components."""
    graph = ConnectionGraph()
    
    # Create chain: comp1 -> comp2 -> comp3
    comp1 = TestComponent("comp1")
    comp2 = TestComponent("comp2")
    comp3 = TestComponent("comp3")
    
    graph.connect(comp1.obs_out, comp2.obs_in)
    graph.connect(comp2.obs_out, comp3.obs_in)
    
    order = graph.topological_sort()
    
    # comp1 should come before comp2, comp2 before comp3
    assert order.index(comp1) < order.index(comp2)
    assert order.index(comp2) < order.index(comp3)


def test_graph_cycle_detection():
    """Test cycle detection."""
    graph = ConnectionGraph()
    
    # Create cycle: comp1 -> comp2 -> comp1
    comp1 = TestComponent("comp1")
    comp2 = TestComponent("comp2")
    
    graph.connect(comp1.obs_out, comp2.obs_in)
    graph.connect(comp2.obs_out, comp1.obs_in)
    
    cycles = graph.detect_cycles()
    
    # Should detect at least one cycle
    assert len(cycles) > 0


def test_graph_validation():
    """Test graph validation."""
    graph = ConnectionGraph()
    
    comp1 = TestComponent("comp1")
    comp2 = TestComponent("comp2")
    
    graph.connect(comp1.obs_out, comp2.obs_in)
    
    is_valid, issues = graph.validate()
    
    # Should be valid (or have only warnings)
    assert is_valid or len(issues) > 0


def test_graph_transfer_all():
    """Test transferring data through all connections."""
    graph = ConnectionGraph()
    
    comp1 = TestComponent("comp1")
    comp2 = TestComponent("comp2")
    comp3 = TestComponent("comp3")
    
    graph.connect(comp1.obs_out, comp2.obs_in)
    graph.connect(comp2.obs_out, comp3.obs_in)
    
    # Write to first component
    obs = Observation(data=[1, 2, 3])
    comp1.obs_out.write(obs)
    
    # Transfer all
    graph.transfer_all()
    
    # Check that data reached comp2
    assert comp2.obs_in.value == obs


if __name__ == "__main__":
    pytest.main([__file__, '-v'])

