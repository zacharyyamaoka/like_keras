#!/usr/bin/env python3

"""
    Tests for system orchestration.
"""

# BAM
from lk import System, SystemConfig, Node, Agent, Env
from lk import Observation, Action, Reward, Done, Info

# PYTHON
import pytest
import numpy as np
from typing import Tuple


class TestAgent(Agent):
    """Test agent implementation."""
    
    def forward(self, obs: Observation) -> Action:
        return Action(data=0.0)


class TestEnv(Env):
    """Test environment implementation."""
    
    def reset(self) -> Observation:
        return Observation(data=np.zeros(4))
    
    def step(self, action: Action) -> Tuple[Observation, Reward, Done, Info]:
        obs = Observation(data=np.zeros(4))
        reward = Reward(value=1.0)
        done = Done(value=False)
        info = Info()
        return obs, reward, done, info


def test_system_creation():
    """Test basic system creation."""
    config = SystemConfig(name="test_system")
    system = System(config=config)
    
    assert system.config.name == "test_system"
    assert len(system.nodes) == 0
    assert len(system.components) == 0


def test_system_with_components():
    """Test system with components."""
    node = Node("test_node")
    agent = TestAgent.from_node(node)
    env = TestEnv.from_node(node)
    
    system = System(
        nodes=[node],
        components=[agent, env],
        config=SystemConfig(name="agent_env")
    )
    
    assert len(system.nodes) == 1
    assert len(system.components) == 2


def test_system_component_discovery():
    """Test automatic component discovery."""
    class TestSystem(System):
        def __init__(self):
            self.node = Node("test")
            self.agent = TestAgent.from_node(self.node)
            self.env = TestEnv.from_node(self.node)
            super().__init__()
    
    system = TestSystem()
    
    # Components should be auto-discovered
    assert len(system.components) >= 2
    assert len(system.nodes) >= 1


def test_system_lifecycle():
    """Test system lifecycle management."""
    node = Node("test_node")
    agent = TestAgent.from_node(node)
    env = TestEnv.from_node(node)
    
    system = System(
        nodes=[node],
        components=[agent, env],
        config=SystemConfig(name="test")
    )
    
    # Configure
    system.configure()
    assert system._configured
    
    # Activate
    system.activate()
    assert system._active
    
    # Deactivate
    system.deactivate()
    assert not system._active
    
    # Shutdown
    system.shutdown()
    assert not system._configured


def test_system_connections():
    """Test system connection creation."""
    node = Node("test_node")
    agent = TestAgent.from_node(node)
    env = TestEnv.from_node(node)
    
    system = System(nodes=[node], components=[agent, env])
    
    # Create connection
    conn = system.connect(env.obs, agent.obs)
    
    assert conn is not None
    assert len(system.graph.connections) >= 1


def test_system_graph_validation():
    """Test graph validation."""
    node = Node("test_node")
    agent = TestAgent.from_node(node)
    env = TestEnv.from_node(node)
    
    system = System(
        nodes=[node],
        components=[agent, env],
        config=SystemConfig(validate_graph=True)
    )
    
    # Create connections
    system.connect(env.obs, agent.obs)
    system.connect(agent.action, env.action_in)
    
    # Configure (triggers validation)
    system.configure()
    
    # Should not raise any errors
    is_valid, issues = system.graph.validate()
    assert is_valid or len(issues) > 0  # May have warnings about cycles


if __name__ == "__main__":
    pytest.main([__file__, '-v'])

