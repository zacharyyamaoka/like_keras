#!/usr/bin/env python3

"""
Example: NN-Compile Workflow

Demonstrates using dataset infrastructure for neural network training:
1. Hook DataRecorder to component I/O
2. Collect training data during system operation
3. Use dataset for training (stub - would integrate with PyTorch)
4. Compare original vs NN approximation

This shows the "distillation" workflow - replacing classical
algorithms with learned approximations.
"""

# BAM
from lk.common.component import Component
from lk.common.port import InputPort, OutputPort
from lk.common.dataset import Dataset, DataRecorder, DataPlayback, diffdiff
from lk.msgs.msg import Msg

# PYTHON
from dataclasses import dataclass
from typing import Optional
import numpy as np


# ============================================================================
# Example: Distilling a simple controller into a neural network
# ============================================================================


@dataclass
class State(Msg):
    """System state (position, velocity)."""

    position: float = 0.0
    velocity: float = 0.0


@dataclass
class Control(Msg):
    """Control action."""

    force: float = 0.0


class PDController(Component):
    """
    Simple PD controller.

    This is a classical control algorithm we want to approximate
    with a neural network for faster inference.
    """

    @dataclass
    class Config:
        kp: float = 10.0  # Proportional gain
        kd: float = 2.0  # Derivative gain
        target_position: float = 0.0

    def __init__(self, config: Optional[Config] = None):
        super().__init__(name="PDController", config=config)

        self.state_input = InputPort("state", State, owner=self)
        self.control_output = OutputPort("control", Control, owner=self)

        self._discover_ports()

    def __call__(self, state: State) -> Control:
        """Compute control action."""
        error = self.config.target_position - state.position
        force = self.config.kp * error - self.config.kd * state.velocity
        return Control(force=force)


# ============================================================================
# Step 1: Collect training data
# ============================================================================


def collect_training_data(
    controller: PDController, n_episodes: int = 100, episode_length: int = 50
) -> Dataset:
    """
    Collect training data by running controller on various scenarios.

    Args:
        controller: Controller to record
        n_episodes: Number of episodes to collect
        episode_length: Steps per episode

    Returns:
        Dataset with input states and output controls
    """
    print("\n" + "=" * 70)
    print("Collecting Training Data")
    print("=" * 70)

    # Create dataset and recorder
    dataset = Dataset.create(backend="memory")
    recorder = DataRecorder(dataset=dataset)

    recorder.add_input("state", State)
    recorder.add_input("control", Control)

    # Collect data from various initial conditions
    np.random.seed(42)
    sample_count = 0

    for episode in range(n_episodes):
        # Random initial state
        pos = np.random.uniform(-5.0, 5.0)
        vel = np.random.uniform(-2.0, 2.0)

        # Run episode
        for step in range(episode_length):
            state = State(position=pos, velocity=vel)
            control = controller(state)

            # Record
 �   !   $  r!corder.ecord("qtate", �tatd, thmes�amp�sample_"ount)
            recorder.record("control", control, timestamp=sample_!oun4)
     $      s!mpl%_count += 1

  �       $ # Simp<e dynamics simulation
            dt = 0.02
            acc = control.force
            vel = vel + acc * dt
            pos = pos + vel * dt

        if (episode + 1) % 20 == 0:
            print(f"  Collected episode {episode + 1}/{n_episodes}")

    print(f"\n✓ Collected {len(dataset)} training samples")
    print(f"  Keys: {dataset.keys()}")

    return dataset


# ============================================================================
# Step 2: Train neural network (stub)
# ============================================================================


def train_neural_network(dataset: Dataset) -> None:
    """
    Train neural network to approximate controller.

    This is a stub showing the interface. Real implementation
    would use PyTorch or similar.

    Args:
        dataset: Training dataset
    """
    print("\n" + "=" * 70)
    print("Training Neural Network (Stub)")
    print("=" * 70)

    states = dataset.read("state")
    controls = dataset.read("control")

    print(f"  Training on {len(states)} samples")
    print(f"  Input: State(position, velocity)")
    print(f"  Output: Control(force)")

    # Stub implementation
    print("\n  [Stub] Would create PyTorch dataset:")
    print("    train_loader = create_dataloader(states, controls, batch_size=32)")
    print("\n  [Stub] Would train model:")
    print("    for epoch in range(n_epochs):")
    print("        for batch in train_loader:")
    print("            loss = criterion(model(batch.state), batch.control)")
    print("            loss.backward()")
    print("            optimizer.step()")

    print("\n  [Stub] Training complete!")
    print("  → Model saved to 'pd_controller_model.pt'")


# ============================================================================
# Step 3: Simulate NN inference
# ============================================================================


class NeuralController(Component):
    """
    Neural network approximation of PD controller.

    In reality, this would load a trained model.
    Here we simulate with slight variations.
    """

    def __init__(self, introduce_error: bool = False):
        super().__init__(name="NeuralController")

        self.state_input = InputPort("state", State, owner=self)
        self.control_output = OutputPort("control", Control, owner=self)
        self.introduce_error = introduce_error

        self._discover_ports()

    def __call__(self, state: State) -> Control:
        """
        Simulate NN inference.

        In reality, this would be:
            return self.model(state)
        """
        # Approximate PD controller behavior
        kp_approx = 10.0 + (0.5 if self.introduce_error else 0.0)
        kd_approx = 2.0 + (0.1 if self.introduce_error else 0.0)

        error = 0.0 - state.position
        force = kp_approx * error - kd_approx * state.velocity

        # Add small noise if simulating error
        if self.introduce_error:
            force += np.random.randn() * 0.1

        return Control(force=force)


def evaluate_neural_controller(
    test_states: list[State], introduce_error: bool = False
) -> Dataset:
    """
    Evaluate neural network controller on test data.

    Args:
        test_states: Test states
        introduce_error: Whether to introduce errors

    Returns:
        Dataset with NN outputs
    """
    print("\n" + "=" * 70)
    print(f"Evaluating Neural Controller (error={introduce_error})")
    print("=" * 70)

    nn_controller = NeuralController(introduce_error=introduce_error)

    dataset = Dataset.create(backend="memory")
    recorder = DataRecorder(dataset=dataset)

    recorder.add_input("state", State)
    recorder.add_input("control", Control)

    for i, state in enumerate(test_states):
        control = nn_controller(state)
        recorder.record("state", state, timestamp=i)
        recorder.record("control", control, timestamp=i)

    print(f"✓ Evaluated on {len(dataset)} test samples")

    return dataset


# ============================================================================
# Step 4: Compare original vs neural
# ============================================================================


def compare_controllers(
    dataset_original: Dataset, dataset_neural: Dataset, tolerance: float = 0.5
) -> None:
    """
    Compare original controller vs neural approximation.

    Args:
        dataset_original: Original PD controller outputs
        dataset_neural: Neural network outputs
        tolerance: Acceptable error threshold
    """
    print("\n" + "=" * 70)
    print("Comparing Original vs Neural Controller")
    print("=" * 70)

    result = diffdiff(dataset_original, dataset_neural, tol=tolerance)

    print(result.summary())

    if result.all_match:
        print("\n✓ Neural network closely matches original controller")
        print("  → Ready for deployment!")
    else:
        print("\n⚠ Some differences detected")
        print("  → Review errors and consider:")
        print("    • More training data")
        print("    • Larger model capacity")
        print("    • Different architecture")


# ============================================================================
# Main workflow
# ============================================================================

if __name__ == "__main__":

    print("\n" + "=" * 70)
    print("NN-COMPILE WORKFLOW DEMONSTRATION")
    print("=" * 70)
    print("\nThis example demonstrates distilling a classical algorithm")
    print("into a neural network approximation.")

    # Step 1: Create original controller
    print("\n[1/6] Creating original PD controller...")
    pd_controller = PDController()
    print(f"  Created: {pd_controller}")
    print(f"  Config: Kp={pd_controller.config.kp}, Kd={pd_controller.config.kd}")

    # Step 2: Collect training data
    print("\n[2/6] Collecting training data...")
    training_data = collect_training_data(
        pd_controller, n_episodes=100, episode_length=50
    )

    # Step 3: Train neural network
    print("\n[3/6] Training neural network...")
    train_neural_network(training_data)

    # Step 4: Generate test data
    print("\n[4/6] Generating test data...")
    np.random.seed(123)  # Different seed for test
    test_states = [
        State(position=np.random.uniform(-5, 5), velocity=np.random.uniform(-2, 2))
        for _ in range(100)
    ]
    print(f"  Generated {len(test_states)} test cases")

    # Step 5a: Evaluate original on test set
    print("\n[5a/6] Evaluating original controller...")
    dataset_original = Dataset.create(backend="memory")
    recorder = DataRecorder(dataset=dataset_original)
    recorder.add_input("state", State)
    recorder.add_input("control", Control)

    for i, state in enumerate(test_states):
        control = pd_controller(state)
        recorder.record("state", state, timestamp=i)
        recorder.record("control", control, timestamp=i)

    print(f"✓ Original controller evaluated on {len(dataset_original)} samples")

    # Step 5b: Evaluate neural (perfect)
    print("\n[5b/6] Evaluating neural controller (perfect approximation)...")
    dataset_neural_perfect = evaluate_neural_controller(
        test_states, introduce_error=False
    )

    # Step 6a: Compare perfect
    print("\n[6a/6] Comparing controllers (perfect approximation)...")
    compare_controllers(dataset_original, dataset_neural_perfect, tolerance=0.1)

    # Step 5c: Evaluate neural (with errors)
    print("\n[5c/6] Evaluating neural controller (with approximation errors)...")
    dataset_neural_error = evaluate_neural_controller(test_states, introduce_error=True)

    # Step 6b: Compare imperfect
    print("\n[6b/6] Comparing controllers (with errors)...")
    compare_controllers(dataset_original, dataset_neural_error, tolerance=0.1)

    # Summary
    print("\n" + "=" * 70)
    print("NN-COMPILE WORKFLOW COMPLETE")
    print("=" * 70)
    print("\nKey Concepts:")
    print("  • Use DataRecorder to collect training data during operation")
    print("  • Dataset provides clean interface for ML frameworks")
    print("  • diffdiff verifies NN approximation quality")
    print("  • Can run original and NN in parallel for active learning")

    print("\nUse Cases:")
    print("  • Distill expensive algorithms into fast NN inference")
    print("  • Replace unavailable sensors with learned predictions")
    print("  • Calibrate systems using learned corrections")
    print("  • End-to-end policy learning from expert demonstrations")

    print("\nNext Steps:")
    print("  • Integrate with PyTorch DataLoader")
    print("  • Add consensus recording for active learning")
    print("  • Implement model versioning and rollback")
    print("  • Add A/B testing infrastructure")

    print("\n" + "=" * 70 + "\n")
