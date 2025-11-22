#!/usr/bin/env python3

"""
RandomAgent - Simple agent that samples random actions.

Useful as a baseline or for testing environments.
"""

# BAM
from dataclasses import dataclass

# PYTHON
from typing import Optional

from lk.agent.agent import Agent
from lk.msgs.data_dist.data_dist import DataDist
from lk.msgs.msg import Action, Observation


class RandomAgent(Agent):
    """
    RandomAgent samples random actions from the action space.

    The action space should be a DataDist distribution.
    """

    @dataclass
    class Config(Agent.Config):
        """
        Configuration for RandomAgent.
        """

        type: str = "random"
        action_space: DataDist | None = None
        seed: int | None = None

    def __init__(
        self,
        name: str | None = None,
        config: Optional["RandomAgent.Config"] = None,
        **kwargs,
    ):
        """
        Initialize random agent.

        Args:
            name: Agent name
            config: Agent configuration
        """
        if config is None:
            config = RandomAgent.Config()

        super().__init__(name=name or "random_agent", config=config, **kwargs)

        # Seed the action space if provided
        if self.config.action_space and self.config.seed is not None:
            self.config.action_space.seed(self.config.seed)

    def forward(self, obs: Observation) -> Action:
        """
        Sample a random action from the action space.

        Args:
            obs: Current observation (ignored by random agent)

        Returns:
            Random action
        """
        if self.config.action_space is None:
            raise ValueError("action_space must be set before calling forward()")

        # Sample from the distribution
        action_data = self.config.action_space.sample()

        return Action(data=action_data)

    @classmethod
    def from_id(cls, agent_id: str, **kwargs) -> "RandomAgent":
        """
        Factory method to create agent from ID.

        Args:
            agent_id: Agent identifier (e.g., "RandomAgent-v1")
            **kwargs: Additional configuration

        Returns:
            RandomAgent instance
        """
        # For now, just create a basic random agent
        # In the future, we can add a registry for different versions
        config = RandomAgent.Config(**kwargs)
        return cls(config=config)


# Register RandomAgent with the top-level Agent registry
def _register_random_agent():
    """Register RandomAgent with Agent registry."""
    from lk.agent.agent import Agent  # Import directly from module, not package

    Agent.register("RandomAgent-v1", RandomAgent)


# Auto-register when module is imported
_register_random_agent()


if __name__ == "__main__":
    # PYTHON
    from lk.msgs.data_dist.int_dist import IntDist

    print("\n" + "=" * 70)
    print("RandomAgent Example")
    print("=" * 70)

    # Create action space (discrete actions 0-4)
    action_space = IntDist.uniform(0, 5)

    # Create random agent
    agent = RandomAgent(config=RandomAgent.Config(action_space=action_space, seed=42))

    print(f"\nAgent: {agent}")
    print(f"Action space: {action_space}")

    # Generate some random actions
    print("\nSampling 10 random actions:")
    dummy_obs = Observation(data=0)
    for i in range(10):
        action = agent.forward(dummy_obs)
        print(f"  Action {i}: {action.data}")

    print("\n" + "=" * 70)
