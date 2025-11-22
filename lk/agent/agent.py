#!/usr/bin/env python3

"""
Agent base class for reinforcement learning.

Agents receive observations and produce actions.
"""

# BAM
from lk.common.component import LifecycleComponent
from lk.common.port import InputPort, OutputPort
from lk.common.node import Node
from lk.msgs.msg import Observation, Action

# PYTHON
from typing import Optional
from dataclasses import dataclass
from abc import abstractmethod


class Agent(LifecycleComponent):
    """
        Base class for agents.

        Agents have:
        - Input: observation
        - Output: action

        Subclasses should implement the forward() or __call__() method.
    """

    # Registry for agent factories
    _registry: dict[str, tuple[type, dict]] = {}

    @classmethod
    def register(cls, agent_id: str, agent_class: type["Agent"], **default_kwargs):
        """
            Register an agent class with an ID.

            Args:
                agent_id: Unique identifier for the agent
                agent_class: Agent class to register
                **default_kwargs: Default kwargs for agent creation
        """
        cls._registry[agent_id] = (agent_class, default_kwargs)

    @classmethod
    def from_id(cls, agent_id: str, **kwargs) -> "Agent":
        """
            Create agent from registered ID.

            This provides a top-level factory similar to Env.from_id().

            Args:
                agent_id: Registered agent ID
                **kwargs: Override default kwargs for agent creation

            Returns:
                Agent instance

            Example:
                agent = Agent.from_id("RandomAgent-v1", action_space=space)
        """
        if agent_id not in cls._registry:
            raise ValueError(
                f"Agent '{agent_id}' not registered. "
                f"Available: {list(cls._registry.keys())}"
            )

        agent_class, default_kwargs = cls._registry[agent_id]
        # Merge default kwargs with provided kwargs
        final_kwargs = {**default_kwargs, **kwargs}

        # Call the specific agent class's from_id if it exists,
        # otherwise create directly
        if hasattr(agent_class, 'from_id') and agent_class != cls:
            return agent_class.from_id(agent_id, **final_kwargs)
        else:
            return agent_class(**final_kwargs)

    @dataclass
    class Config:
        """
        Configuration for agents.

        Subclasses can override to add agent-specific configuration.
        """

        type: str = "base"

    def __init__(
        self,
        name: Optional[str] = None,
        node: Optional[Node] = None,
        config: Optional["Agent.Config"] = None,
    ):
        """
        Initialize agent.

        Args:
            name: Agent identifier
            node: Node this agent belongs to
            config: Agent configuration
        """
        super().__init__(name=name or "agent", node=node, config=config)

        # Define ports
        self.obs = InputPort("obs", Observation, owner=self)
        self.action = OutputPort("action", Action, owner=self)

        # Re-discover ports after defining them
        self._discover_ports()

    @classmethod
    def from_node(
        cls,
        node: Node,
        config: Optional["Agent.Config"] = None,
        component_id: str = "agent",
    ) -> "Agent":
        """
        Factory method to create agent from node.

        Args:
            node: Node to attach agent to
            config: Agent configuration (overrides node config)
            component_id: ID to lookup config in node.component_configs

        Returns:
            Agent instance
        """
        # Check node for config if not provided
        if config is None:
            config = node.get_component_config(component_id)

        agent = cls(name=component_id, node=node, config=config)
        node.add_component(agent)
        return agent

    @abstractmethod
    def forward(self, obs: Observation) -> Action:
        """
        Compute action from observation.

        Override this method to implement agent logic.

        Args:
            obs: Current observation

        Returns:
            Action to take
        """
        raise NotImplementedError("Subclasses must implement forward()")

    def __call__(self, obs_port: Optional[InputPort] = None) -> OutputPort[Action]:
        """
            PyTorch/Keras-like callable interface.

            Enables syntax like:
                action_port = agent(env.obs)  # Using port
                action = agent.forward(obs)    # Using value

            Args:
                obs_port: Optional observation port to connect to and read from

            Returns:
                Action output port
        """
        # If an observation port is provided, connect it and process
        if obs_port is not None:
            from lk.common.graph import Connection

            # Connect the ports
            Connection(source=obs_port, target=self.obs)

            # Read the observation and compute action
            obs_value = obs_port.value
            if obs_value is not None:
                action = self.forward(obs_value)
                self.action.write(action)

        return self.action

    def on_activate(self):
        """Called when agent is activated."""
        # Agents typically don't need to do anything on activate
        # They respond to observations
        pass

    def __repr__(self) -> str:
        return f"Agent(name={self.name}, state={self.lifecycle_state.value})"
