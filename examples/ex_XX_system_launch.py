#!/usr/bin/env python3

"""
    System Launch File Examples
    
    Demonstrates multiple API styles for defining systems:
    1. Functional factory (simple)
    2. Functional factory with typed config
    3. Functional factory with explicit config
    4. Class-based with config dataclass
    5. Class-based with full type hints (RECOMMENDED)
"""

# BAM
from lk import System, SystemConfig, Node, Agent, Env

# PYTHON
from typing import Optional
from dataclasses import dataclass


# =============================================================================
# Approach 1: Functional Factory (Simple)
# =============================================================================
def generate_system_functional_simple(system_config: Optional[SystemConfig] = None) -> System:
    """
        Simple functional approach with optional config.
        
        Good for quick prototyping and simple systems.
    """
    # First define nodes
    node = Node(name="agent_env_loop")
    
    # Define Components and assign them to nodes
    agent = Agent.from_node(node)
    env = Env.from_node(node)
    
    # Define Connections between components
    # The callable interface creates connections automatically
    action = agent(env.obs)  # agent receives env.obs, returns action port
    obs = env(action)        # env receives action, returns obs port
    
    # Create a system
    # inputs/outputs become "public APIs" for introspection
    return System(
        nodes=[node], 
        components=[agent, env], 
        inputs=[], 
        outputs=[action, obs], 
        subsystems=[], 
        config=system_config
    )


# =============================================================================
# Approach 2: Functional Factory with Typed Config
# =============================================================================
@dataclass
class BcMedWasteConfig(SystemConfig):
    """Custom system configuration."""
    agent_type: str = "bc"
    env_type: str = "mujoco"
    episode_length: int = 1000


def generate_system_typed_config(system_config: BcMedWasteConfig) -> System:
    """
        Functional approach with strongly-typed configuration.
        
        Good when you want type safety for configs.
    """
    node = Node(name="agent_env_loop")
    
    # Note: In real implementation, these would use the config
    agent = Agent.from_node(node)
    env = Env.from_node(node)
    
    action = agent(env.obs)
    obs = env(action)
    
    return System(
        nodes=[node], 
        components=[agent, env], 
        inputs=[], 
        outputs=[action, obs], 
        subsystems=[], 
        config=system_config
    )


# =============================================================================
# Approach 3: Functional with Explicit Config
# =============================================================================
def generate_system_explicit_config() -> System:
    """
        Functional approach with config defined in function.
        
        Good for one-off systems with specific configurations.
    """
    # Feel free to change the call of this function to whatever you want!
    my_favorite_system_config = SystemConfig(
        name="my_system",
        rate_hz=100.0,
        verbose=True
    )
    
    # First define nodes
    node = Node(name="agent_env_loop")
    
    # Define Components and assign them to nodes
    agent = Agent.from_node(node)
    env = Env.from_node(node)
    
    # Define Connections between components
    action = agent(env.obs)
    obs = env(action)
    
    # Create a system
    return System(
        nodes=[node], 
        components=[agent, env], 
        inputs=[], 
        outputs=[action, obs], 
        subsystems=[]
    )


# =============================================================================
# Approach 4: Class-based with Config Dataclass
# =============================================================================
@dataclass
class MyFavoriteSystemConfig(SystemConfig):
    """Configuration for custom system."""
    node_name: str = "agent_env_loop"
    agent_name: str = "agent"
    env_name: str = "env"
    action_name: str = "action"
    obs_name: str = "obs"


def generate_system_class_based() -> System:
    """
        OOP approach with nested class definition.
        
        Good for encapsulation and complex systems.
    """
    class MyFavoriteSystem(System):
        def __init__(self, config: MyFavoriteSystemConfig):
            # Initialize parent first (will call _discover_components_and_nodes)
            super().__init__(config=config)
            
            # Define components (will be auto-discovered by parent)
            node = Node(name="agent_env_loop")
            agent = Agent.from_node(node)
            env = Env.from_node(node)
            
            # Create connections
            action = agent(env.obs)
            obs = env(action)
            
            # Store for potential introspection
            self.node = node
            self.agent = agent
            self.env = env
            self.action = action
            self.obs = obs
    
    return MyFavoriteSystem(config=MyFavoriteSystemConfig())


# =============================================================================
# Approach 5: Class-based with Full Type Hints (RECOMMENDED)
# =============================================================================
class AgentEnvSystem(System):
    """
        Clean class-based system with full type hints.
        
        RECOMMENDED: This is the most maintainable and IDE-friendly approach.
        - Type hints enable autocomplete
        - Clear attribute access (system.agent, system.env)
        - Easy to subclass and extend
        - Components auto-discovered by System base class
    """
    
    def __init__(self, config: Optional[MyFavoriteSystemConfig] = None):
        # Components defined as instance attributes BEFORE super().__init__
        # so they can be auto-discovered
        
        # Create components first
        self.node = Node(name="agent_env_loop")
        self.agent = Agent.from_node(self.node)
        self.env = Env.from_node(self.node)
        
        # Initialize parent (auto-discovers components)
        super().__init__(config=config or MyFavoriteSystemConfig())
        
        # Define connections
        # Note: These create Connection objects automatically
        self.action = self.agent(self.env.obs)
        self.obs = self.env(self.action)


def generate_system_recommended() -> System:
    """Generate system using recommended approach."""
    return AgentEnvSystem(config=MyFavoriteSystemConfig(
        name="recommended_system",
        verbose=True
    ))


# =============================================================================
# Main Entry Point
# =============================================================================
if __name__ == "__main__":
    # Choose which approach to test
    # Uncomment the one you want to try:
    
    # system = generate_system_functional_simple()
    # system = generate_system_typed_config(BcMedWasteConfig())
    # system = generate_system_explicit_config()
    # system = generate_system_class_based()
    system = generate_system_recommended()
    
    # Launch the system
    # Note: This will fail until we implement concrete Agent/Env subclasses
    # See ex_01_simple_loop.py for a working example
    print(f"Created {system}")
    print(f"  Nodes: {len(system.nodes)}")
    print(f"  Components: {len(system.components)}")
    print(f"  Connections: {len(system.graph.connections)}")
    
    # To run:
    # system.launch(iterations=10)
