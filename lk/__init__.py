"""
    Like Keras (lk) - A robotics learning framework
    
    A PyTorch-inspired framework for building robotic learning systems.
"""

# BAM
from lk.common.system import System, SystemConfig
from lk.common.component import Component, LifecycleComponent, ComponentLifecycleMixin, LifecycleState
from lk.common.node import Node, NodeConfig, ExecutionMode
from lk.common.port import Port, InputPort, OutputPort, PortCollection
from lk.common.graph import Connection, ConnectionGraph
from lk.agent.agent import Agent
from lk.env.env import Env
from lk.msgs.msg import Msg, Observation, Action, Reward, Done, Info

# Public API
__all__ = [
    # System
    'System',
    'SystemConfig',
    
    # Components
    'Component',
    'LifecycleComponent',
    'ComponentLifecycleMixin',
    'LifecycleState',
    
    # Nodes
    'Node',
    'NodeConfig',
    'ExecutionMode',
    
    # Ports
    'Port',
    'InputPort',
    'OutputPort',
    'PortCollection',
    
    # Graph
    'Connection',
    'ConnectionGraph',
    
    # Agent & Environment
    'Agent',
    'Env',
    
    # Messages
    'Msg',
    'Observation',
    'Action',
    'Reward',
    'Done',
    'Info',
]

