#!/usr/bin/env python3

"""
Objective Function Components for RL systems.

Self-contained components following ROS-style architecture:
- All logic lives inside components
- System space only wires ports
- Subclass for custom objectives
"""

# BAM
# PYTHON
from dataclasses import dataclass, field

from lk.common.component import Component
from lk.common.port import InputPort, OutputPort
from lk.msgs.msg import Msg, Reward


@dataclass
class ScalarValue(Msg):
    """Single scalar value message."""

    value: float


@dataclass
class ObjectiveResult(Msg):
    """
    Result of objective function evaluation.
    """

    fitness: float
    step_count: int = 0
    components: dict[str, float] = field(default_factory=dict)


class ObjectiveFunction(Component):
    """
    Base class for objective functions.

    Subclass this to create custom objectives.
    All complexity stays INSIDE the component (complex Python ok!).
    System space only wires ports (clean!).
    """

    @dataclass
    class Config(Component.Config):
        """Base configuration for objective functions."""

        pass

    def __init__(self, name: str | None = None, config: Config | None = None):
        """
        Initialize objective function.

        Args:
            name: Component name
            config: Component configuration
        """
        if config is None:
            config = self.Config()

        super().__init__(name=name or "objective_fn", config=config)

    def update(self):
        """
        Update objective function.

        Component controls its own execution.
        Override this in subclasses.
        """
        raise NotImplementedError(f"{self.__class__.__name__} must implement update()")


class RewardObjective(ObjectiveFunction):
    """
    Cumulative reward tracking objective.

    Most common RL objective: maximize total reward.

    Ports:
        reward_in (InputPort): Subscribe to reward signal
        fitness (OutputPort): Cumulative reward fitness score
    """

    def __init__(self, name: str | None = None):
        """Initialize reward objective."""
        super().__init__(name=name or "reward_objective")

        # Input port
        self.reward_in = InputPort("reward_in", Reward, owner=self)

        # Output port
        self.fitness = OutputPort("fitness", ScalarValue, owner=self)

        # Re-discover ports
        self._discover_ports()

        # Internal state
        self.total_reward = 0.0
        self.step_count = 0

    def update(self):
        """Update cumulative reward."""
        # Read from input port
        reward_msg = self.reward_in.value

        if reward_msg is not None:
            # Accumulate reward
            self.total_reward += reward_msg.value
            self.step_count += 1

            # Write to output port
            self.fitness.write(ScalarValue(value=self.total_reward))

    def summary(self) -> str:
        """Get summary of objective."""
        avg_reward = self.total_reward / self.step_count if self.step_count > 0 else 0
        return f"""
Reward Objective Summary:
  Steps: {self.step_count}
  Total Reward: {self.total_reward:.2f}
  Average Reward: {avg_reward:.3f}
        """.strip()


class WeightedSumObjective(ObjectiveFunction):
    """
    Multi-objective with weighted sum.

    Combines multiple inputs with specified weights.

    Config:
        input_names: Names for each input port
        weights: Weight for each input (must match input_names length)
        require_all_inputs: Error if any input is missing

    Example:
        obj = WeightedSumObjective(
            config=WeightedSumObjective.Config(
                input_names=["reward", "efficiency"],
                weights=[0.85, 0.15]
            )
        )
        obj.reward.connect(env.reward)
        obj.efficiency.connect(agent.efficiency)
    """

    @dataclass
    class Config(ObjectiveFunction.Config):
        """Configuration for weighted sum objective."""

        input_names: list[str] = field(default_factory=list)
        weights: list[float] = field(default_factory=list)
        require_all_inputs: bool = True

    def __init__(self, config: Config | None = None, **kwargs):
        """
        Initialize weighted sum objective.

        Can pass config or kwargs:
            WeightedSumObjective(
                config=Config(input_names=["a", "b"], weights=[1.0, 2.0])
            )
        or:
            WeightedSumObjective(input_names=["a", "b"], weights=[1.0, 2.0])
        """
        # Handle kwargs
        if config is None and kwargs:
            config = WeightedSumObjective.Config(**kwargs)
        elif config is None:
            config = WeightedSumObjective.Config()

        super().__init__(name="weighted_objective", config=config)

        # Validate config
        if not self.config.input_names or not self.config.weights:
            raise ValueError("Must provide input_names and weights")

        if len(self.config.input_names) != len(self.config.weights):
            raise ValueError(
                f"Number of inputs ({len(self.config.input_names)}) must match "
                f"number of weights ({len(self.config.weights)})"
            )

        # Dynamically create input ports
        for input_name in self.config.input_names:
            port = InputPort(input_name, ScalarValue, owner=self)
            setattr(self, input_name, port)

        # Output port
        self.fitness = OutputPort("fitness", ObjectiveResult, owner=self)

        # Re-discover ports
        self._discover_ports()

        # Internal state
        self.total_fitness = 0.0
        self.step_count = 0

    def update(self):
        """Compute weighted sum of inputs."""
        # Read from all input ports
        values = {}
        for input_name in self.config.input_names:
            port = getattr(self, input_name)
            value_msg = port.value

            if value_msg is None:
                if self.config.require_all_inputs:
                    raise ValueError(f"Missing required input: {input_name}")
                else:
                    values[input_name] = 0.0
            else:
                values[input_name] = value_msg.value

        # Compute weighted sum
        weighted_sum = sum(
            values[name] * weight
            for name, weight in zip(self.config.input_names, self.config.weights)
        )

        self.total_fitness += weighted_sum
        self.step_count += 1

        # Write to output port with breakdown
        result = ObjectiveResult(
            fitness=self.total_fitness, step_count=self.step_count, components=values
        )
        self.fitness.write(result)


class ThresholdObjective(ObjectiveFunction):
    """
    Threshold-based objective for early stopping.

    Monitors if a value crosses a threshold.
    Useful for pruning poor performers early.

    Config:
        threshold: Threshold value
        comparison: Comparison operator ('>', '<', '>=', '<=')
        window_size: Number of steps to average over

    Ports:
        value_in (InputPort): Value to monitor
        should_stop (OutputPort): True if threshold crossed
        fitness (OutputPort): Current average value
    """

    @dataclass
    class Config(ObjectiveFunction.Config):
        """Configuration for threshold objective."""

        threshold: float = 0.0
        comparison: str = "<"  # '>', '<', '>=', '<='
        window_size: int = 100

    def __init__(self, config: Config | None = None, **kwargs):
        """Initialize threshold objective."""
        if config is None and kwargs:
            config = ThresholdObjective.Config(**kwargs)
        elif config is None:
            config = ThresholdObjective.Config()

        super().__init__(name="threshold_objective", config=config)

        # Validate comparison
        if self.config.comparison not in [">", "<", ">=", "<="]:
            raise ValueError(f"Invalid comparison: {self.config.comparison}")

        # Input port
        self.value_in = InputPort("value_in", ScalarValue, owner=self)

        # Output ports
        self.should_stop = OutputPort("should_stop", ScalarValue, owner=self)
        self.fitness = OutputPort("fitness", ScalarValue, owner=self)

        # Re-discover ports
        self._discover_ports()

        # Internal state
        self.values = []
        self.step_count = 0

    def update(self):
        """Check if threshold is crossed."""
        # Read from input port
        value_msg = self.value_in.value

        if value_msg is None:
            return

        # Store value
        self.values.append(value_msg.value)
        self.step_count += 1

        # Keep only recent values
        if len(self.values) > self.config.window_size:
            self.values.pop(0)

        # Compute average
        avg_value = sum(self.values) / len(self.values)

        # Check threshold
        if self.config.comparison == ">":
            should_stop = avg_value > self.config.threshold
        elif self.config.comparison == "<":
            should_stop = avg_value < self.config.threshold
        elif self.config.comparison == ">=":
            should_stop = avg_value >= self.config.threshold
        else:  # '<='
            should_stop = avg_value <= self.config.threshold

        # Write to output ports
        self.should_stop.write(ScalarValue(value=1.0 if should_stop else 0.0))
        self.fitness.write(ScalarValue(value=avg_value))


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("Objective Function Components")
    print("=" * 70)

    # Example: RewardObjective
    print("\n1. RewardObjective - Simple cumulative reward")
    reward_obj = RewardObjective()
    print(
        f"   Ports: {list(reward_obj.inputs._ports.keys())} → {list(reward_obj.outputs._ports.keys())}"
    )

    # Example: WeightedSumObjective
    print("\n2. WeightedSumObjective - Multi-objective with weights")
    weighted_obj = WeightedSumObjective(
        input_names=["reward", "efficiency", "exploration"], weights=[1.0, 0.5, 0.2]
    )
    print(f"   Inputs: {list(weighted_obj.inputs._ports.keys())}")
    print(f"   Weights: {weighted_obj.config.weights}")

    # Example: ThresholdObjective
    print("\n3. ThresholdObjective - Early stopping")
    threshold_obj = ThresholdObjective(threshold=0.3, comparison="<", window_size=50)
    print(
        f"   Threshold: {threshold_obj.config.threshold} ({threshold_obj.config.comparison})"
    )
    print(f"   Window: {threshold_obj.config.window_size}")

    print("\n" + "=" * 70)
    print("All components are self-contained!")
    print("System space only wires ports (clean architecture)")
    print("=" * 70 + "\n")
