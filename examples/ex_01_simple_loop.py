#!/usr/bin/env python3

"""
Simple Agent-Environment Loop Example

Demonstrates a complete working system with:
- Concrete Agent implementation (random actions)
- Concrete Environment implementation (simple CartPole-like)
- Full lifecycle phases
- Visualization of lifecycle transitions
"""

# BAM
from lk import System, Node, Agent, Env
from lk import Observation, Action, Reward, Done, Info

# PYTHON
import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass


# =============================================================================
# Simple Environment Implementation
# =============================================================================


class SimpleCartEnv(Env):
    """
    Simple CartPole-like environment.

    State: [x, x_dot, theta, theta_dot]
    Action: force (left/right)
    Goal: Keep pole upright
    """

    @dataclass
    class Config:
        """
        Configuration for simple environment.
        """

        type: str = "simple_cart"
        max_steps: int = 100
        state_dim: int = 4  # [position, velocity, angle, angular_velocity]

    def __init__(
        self,
        name: Optional[str] = None,
        node: Optional[Node] = None,
        config: Optional["SimpleCartEnv.Config"] = None,
    ):
        """Initialize simple cart environment."""
        super().__init__(name=name, node=node, config=config)

        # State
        self.state = np.zeros(self.config.state_dim)
        self.steps = 0

    def reset(self) -> Observation:
        """
        Reset environment to initial state.
        """
        # Random initial state
        self.state = np.random.uniform(-0.05, 0.05, self.config.state_dim)
        self.steps = 0

        print(f"  {self.name}: reset() called")

        return Observation(
            data=self.state.copy(), shape=self.state.shape, dtype=str(self.state.dtype)
        )

    def step(self, action: Action) -> Tuple[Observation, Reward, Done, Info]:
        """
        Take a step in the environment.

        Simple dynamics: cart moves based on action
        """
        force = action.data if isinstance(action.data, (int, float)) else 0.0

        # Simple dynamics (not physically accurate, just for demo)
        dt = 0.02
        x, x_dot, theta, theta_dot = self.state

        # Update state
        x_dot += force * dt
        x += x_dot * dt
        theta_dot += np.sin(theta) * dt - force * 0.1 * dt
        theta += theta_dot * dt

        self.state = np.array([x, x_dot, theta, theta_dot])
        self.steps += 1

        # Compute reward (keep pole upright, stay near center)
        reward_val = 1.0 - abs(theta) - 0.1 * abs(x)

        # Check if done
        done_val = (
            abs(x) > 2.4 or abs(theta) > 0.5 or self.steps >= self.config.max_steps
        )

        obs = Observation(data=self.state.copy())
        reward = Reward(value=float(reward_val))
        done = Done(value=bool(done_val))
        info = Info(data={"steps": self.steps})

        return obs, reward, done, info


# =============================================================================
# Simple Agent Implementation
# =============================================================================


class RandomAgent(Agent):
    """
    Agent that takes random actions.

    Simple baseline for testing.
    """

    @dataclass
    class Config:
        """
        Configuration for random agent.
        """

        type: str = "random_agent"
        action_range: Tuple[float, float] = (-10.0, 10.0)

    def __init__(
        self,
        name: Optional[str] = None,
        node: Optional[Node] = None,
        config: Optional["RandomAgent.Config"] = None,
    ):
        """Initialize random agent."""
        super().__init__(name=name, node=node, config=config)

        print(f"  {self.name}: created")

    def forward(self, obs: Observation) -> Action:
        """
        Compute random action.
        """
        # Random action in range
        action_val = np.random.uniform(
            self.config.action_range[0], self.config.action_range[1]
        )

        return Action(data=float(action_val), action_space="continuous")

    def on_configure(self):
        """Called during configuration phase."""
        print(f"  {self.name}: configure() called")

    def on_activate(self):
        """Called during activation phase."""
        print(f"  {self.name}: activate() called")


# =============================================================================
# System Definition (Using Recommended Class-based Approach)
# =============================================================================
@dataclass
class SimpleLoopConfig(System.Config):
    """Configuration for simple agent-env loop."""

    name: str = "simple_loop"
    rate_hz: float = 50.0  # 50 Hz
    max_iterations: int = 100
    verbose: bool = True


class SimpleLoopSystem(System):
    """
    Simple agent-environment loop system.

    Demonstrates:
    - Component lifecycle management
    - Port-based communication
    - Sequential execution
    """

    def __init__(self, config: Optional[SimpleLoopConfig] = None):
        """Initialize simple loop system."""
        # Create components BEFORE super().__init__ for auto-discovery
        self.node = Node(
            name="main_loop",
            component_configs={
                "env": SimpleCartEnv.Config(max_steps=100),
                "agent": RandomAgent.Config(),
            },
        )

        self.env = SimpleCartEnv.from_node(self.node, component_id="env")
        self.agent = RandomAgent.from_node(self.node, component_id="agent")

        # Initialize parent (auto-discovers components)
        super().__init__(config=config or SimpleLoopConfig())

        # Create connections
        # Agent receives observations from environment
        # Environment receives actions from agent
        self.action_port = self.agent(self.env.obs)
        self.obs_port = self.env(self.action_port)

        # Expose outputs for introspection
        self.add_output(self.action_port)
        self.add_output(self.obs_port)

    def step(self):
        """
        Custom step implementation with actual execution.
        """
        if not self._active:
            raise RuntimeError("System must be activated before stepping")

        # Read current observation from environment
        obs = self.env.obs.value

        # Agent computes action
        if obs is not None:
            action = self.agent.forward(obs)
            self.agent.action.write(action)

            # Transfer action to environment
            self.env.action_in.value = action

            # Environment steps
            new_obs, reward, done, info = self.env.step(action)

            # Update environment outputs
            self.env.obs.write(new_obs)
            self.env.reward.write(reward)
            self.env.done.write(done)
            self.env.info.write(info)

            # Print status every 10 steps
            if self._iteration % 10 == 0:
                print(
                    f"Step {self._iteration}: reward={reward.value:.3f}, done={done.value}"
                )

            # Reset if done
            if done.value:
                print(f"Episode finished at step {self._iteration}. Resetting...")
                reset_obs = self.env.reset()
                self.env.obs.write(reset_obs)

        self._iteration += 1


# =============================================================================
# Main Entry Point
# =============================================================================
if __name__ == "__main__":
    import time

    print("=" * 70)
    print("Simple Agent-Environment Loop Example")
    print("=" * 70)

    # Create system
    print("\n1. Creating system...")
    config = SimpleLoopConfig(
        name="simple_loop_demo",
        rate_hz=10.0,  # Slow for visualization
        max_iterations=100,
        verbose=True,
    )
    system = SimpleLoopSystem(config=config)

    print(f"\nSystem created: {system}")
    print(f"  Nodes: {len(system.nodes)}")
    print(f"  Components: {len(system.components)}")
    print(f"  Connections: {len(system.graph.connections)}")

    # Show lifecycle transitions
    print("\n2. Lifecycle Transitions:")
    print("  State: UNCONFIGURED")

    print("\n  Calling configure()...")
    system.configure()
    print("  State: INACTIVE (configured)")

    print("\n  Calling activate()...")
    system.activate()
    print("  State: ACTIVE")

    # Run for a few iterations
    print("\n3. Running system...")
    print("-" * 70)
    try:
        system.run(iterations=100)
    except KeyboardInterrupt:
        print("\nInterrupted by user")

    # Shutdown
    print("-" * 70)
    print("\n4. Shutting down...")
    system.deactivate()
    print("  State: INACTIVE (deactivated)")

    system.shutdown()
    print("  State: UNCONFIGURED")

    print("\n" + "=" * 70)
    print("Example completed successfully!")
    print("=" * 70)
