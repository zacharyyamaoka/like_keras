#!/usr/bin/env python3

"""
Example: Complete AI-Compile Workflow

Demonstrates the full workflow for verifying cross-language implementations:
1. Define a component to compile
2. Generate verification dataset
3. Create compilation prompt
4. Record original implementation
5. Record new implementation (simulated)
6. Compare using diffdiff

This example shows how to use the dataset infrastructure
for AI-compile verification workflows.
"""

# BAM
from lk.common.component import Component
from lk.common.port import InputPort, OutputPort
from lk.common.dataset import Dataset, DataRecorder, DataPlayback, diffdiff
from lk.msgs.msg import Msg
from lk.utils.prompt import create_prompt, send_prompt

# PYTHON
from dataclasses import dataclass
from typing import Optional
import numpy as np
from pathlib import Path


# ============================================================================
# Step 1: Define a simple component to compile
# ============================================================================


@dataclass
class Vec3(Msg):
    """Simple 3D vector message."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "Vec3":
        return cls(x=float(arr[0]), y=float(arr[1]), z=float(arr[2]))


class VectorNormalizer(Component):
    """
    Component that normalizes 3D vectors.

    This is a simple example component we want to compile to C++.
    """

    @dataclass
    class Config:
        min_length: float = 1e-6  # Minimum length to avoid division by zero

    def __init__(self, name: Optional[str] = None, config: Optional[Config] = None):
        super().__init__(name=name or "VectorNormalizer", config=config)

        # Define ports
        self.input = InputPort("input", Vec3, owner=self)
        self.output = OutputPort("output", Vec3, owner=self)

        self._discover_ports()

    def __call__(self, vec: Vec3) -> Vec3:
        """Normalize input vector."""
        arr = vec.to_array()
        length = np.linalg.norm(arr)

        if length < self.config.min_length:
            # Return zero vector for very small inputs
            return Vec3(0.0, 0.0, 0.0)

        normalized = arr / length
        return Vec3.from_array(normalized)


# ============================================================================
# Step 2: Generate test dataset
# ============================================================================


def generate_test_inputs(n_samples: int = 100) -> list[Vec3]:
    """
    Generate random test vectors.

    Args:
        n_samples: Number of test cases

    Returns:
        List of Vec3 test inputs
    """
    np.random.seed(42)  # Reproducibility

    inputs = []
    for _ in range(n_samples):
        # Mix of different vector types
        vec_type = np.random.choice(["random", "unit", "zero", "large"])

        if vec_type == "random":
            vec = Vec3(x=np.random.randn(), y=np.random.randn(), z=np.random.randn())
        elif vec_type == "unit":
            # Already normalized
            theta = np.random.uniform(0, 2 * np.pi)
            phi = np.random.uniform(0, np.pi)
            vec = Vec3(
                x=np.sin(phi) * np.cos(theta),
                y=np.sin(phi) * np.sin(theta),
                z=np.cos(phi),
            )
        elif vec_type == "zero":
            # Test edge case
            vec = Vec3(0.0, 0.0, 0.0)
        else:  # large
            vec = Vec3(
                x=np.random.randn() * 1000,
                y=np.random.randn() * 1000,
                z=np.random.randn() * 1000,
            )

        inputs.append(vec)

    return inputs


# ============================================================================
# Step 3: Record original implementation
# ============================================================================


def record_original_implementation(
    component: VectorNormalizer, inputs: list[Vec3]
) -> Dataset:
    """
    Record outputs of original Python implementation.

    Args:
        component: Original component
        inputs: Test inputs

    Returns:
        Dataset with recorded inputs and outputs
    """
    print("\n" + "=" * 70)
    print("Recording Original Implementation")
    print("=" * 70)

    # Create dataset and recorder
    dataset = Dataset.create(backend="memory")
    recorder = DataRecorder(dataset=dataset)

    # Add ports
    recorder.add_input("input", Vec3)
    recorder.add_input("output", Vec3)

    # Process and record
    for i, input_vec in enumerate(inputs):
        output_vec = component(input_vec)
        recorder.record("input", input_vec, timestamp=i)
        recorder.record("output", output_vec, timestamp=i)

    print(f"✓ Recorded {len(dataset)} samples")
    print(f"  Keys: {dataset.keys()}")

    return dataset


# ============================================================================
# Step 4: Create compilation prompt
# ============================================================================


def create_compilation_prompt(
    component: VectorNormalizer, dataset_path: Optional[Path] = None
) -> None:
    """
    Generate prompt for AI-assisted compilation.

    Args:
        component: Component to compile
        dataset_path: Path to verification dataset
    """
    print("\n" + "=" * 70)
    print("Creating Compilation Prompt")
    print("=" * 70)

    prompt = create_prompt(
        component=component,
        target_language="cpp",
        dataset_path=dataset_path,
        description="""
        Normalize 3D vectors to unit length.
        Handle edge cases: zero vectors, very small vectors.
        Should compile with C++17 and use Eigen for linear algebra.
        """,
        dependencies=["Eigen3"],
        build_instructions="""
        # CMakeLists.txt
        find_package(Eigen3 REQUIRED)
        add_executable(vector_normalizer vector_normalizer.cpp)
        target_link_libraries(vector_normalizer Eigen3::Eigen)
        """,
        notes="Pay special attention to the min_length threshold handling.",
    )

    print("\nPrompt preview (first 500 chars):")
    print(prompt.to_markdown()[:500] + "...")

    # Optionally save to file
    # prompt.save('compile_vector_normalizer.md')
    # print(f"\n✓ Saved prompt to compile_vector_normalizer.md")


# ============================================================================
# Step 5: Simulate new implementation and record
# ============================================================================


def record_new_implementation(
    inputs: list[Vec3], introduce_error: bool = False
) -> Dataset:
    """
    Simulate recording from new C++ implementation.

    In practice, this would be actual outputs from compiled C++ code.
    Here we simulate with slight variations.

    Args:
        inputs: Test inputs
        introduce_error: If True, introduce small errors to test diffdiff

    Returns:
        Dataset with recorded outputs
    """
    print("\n" + "=" * 70)
    print(f"Recording New Implementation (error={introduce_error})")
    print("=" * 70)

    # Create dataset and recorder
    dataset = Dataset.create(backend="memory")
    recorder = DataRecorder(dataset=dataset)

    recorder.add_input("input", Vec3)
    recorder.add_input("output", Vec3)

    # Simulate processing (in reality, this would be C++ executable)
    for i, input_vec in enumerate(inputs):
        # Simulate normalization
        arr = input_vec.to_array()
        length = np.linalg.norm(arr)

        if length < 1e-6:
            output_arr = np.array([0.0, 0.0, 0.0])
        else:
            output_arr = arr / length

            # Optionally introduce small error
            if introduce_error:
                output_arr += np.random.randn(3) * 1e-4

        output_vec = Vec3.from_array(output_arr)

        recorder.record("input", input_vec, timestamp=i)
        recorder.record("output", output_vec, timestamp=i)

    print(f"✓ Recorded {len(dataset)} samples")

    return dataset


# ============================================================================
# Step 6: Compare implementations
# ============================================================================


def compare_implementations(
    dataset_original: Dataset, dataset_new: Dataset, tolerance: float = 1e-6
) -> None:
    """
    Compare original and new implementations using diffdiff.

    Args:
        dataset_original: Reference dataset
        dataset_new: New implementation dataset
        tolerance: Comparison tolerance
    """
    print("\n" + "=" * 70)
    print("Comparing Implementations")
    print("=" * 70)

    result = diffdiff(dataset_original, dataset_new, tol=tolerance)

    print(result.summary())

    if result.all_match:
        print("\n✓✓✓ SUCCESS! New implementation matches original ✓✓✓")
    else:
        print("\n✗✗✗ FAILURE! Implementations differ ✗✗✗")
        print("\nNext steps:")
        print("  1. Review differences in output")
        print("  2. Check implementation logic")
        print("  3. Verify edge case handling")
        print("  4. Re-run comparison after fixes")


# ============================================================================
# Step 7: Demonstrate playback
# ============================================================================


def demonstrate_playback(dataset: Dataset) -> None:
    """
    Demonstrate dataset playback capability.

    Args:
        dataset: Dataset to play back
    """
    print("\n" + "=" * 70)
    print("Demonstrating Playback")
    print("=" * 70)

    # Create playback component
    playback = DataPlayback(dataset=dataset)

    print(f"Playback component: {playback}")
    print(f"Output ports: {list(playback.outputs._ports.keys())}")

    # Play first 5 samples
    print("\nPlaying back first 5 samples:")
    for i in range(min(5, len(dataset))):
        playback.step()
        values = playback.get_current_values()

        input_vec = values["input"]
        output_vec = values["output"]

        print(f"  Sample {i}:")
        print(f"    Input:  ({input_vec.x:.3f}, {input_vec.y:.3f}, {input_vec.z:.3f})")
        print(
            f"    Output: ({output_vec.x:.3f}, {output_vec.y:.3f}, {output_vec.z:.3f})"
        )


# ============================================================================
# Main workflow
# ============================================================================

if __name__ == "__main__":

    print("\n" + "=" * 70)
    print("AI-COMPILE WORKFLOW DEMONSTRATION")
    print("=" * 70)
    print("\nThis example demonstrates the complete workflow for verifying")
    print("cross-language implementations using dataset recording and playback.")

    # Step 1: Create component
    print("\n[1/7] Creating component to compile...")
    component = VectorNormalizer()
    print(f"  Created: {component}")

    # Step 2: Generate test inputs
    print("\n[2/7] Generating test dataset...")
    test_inputs = generate_test_inputs(n_samples=50)
    print(f"  Generated {len(test_inputs)} test cases")

    # Step 3: Record original
    dataset_original = record_original_implementation(component, test_inputs)

    # Step 4: Create prompt
    create_compilation_prompt(component)

    # Step 5a: Record perfect new implementation
    print("\n" + "=" * 70)
    print("SCENARIO A: Perfect Implementation")
    print("=" * 70)
    dataset_new_perfect = record_new_implementation(test_inputs, introduce_error=False)

    # Step 6a: Compare perfect
    compare_implementations(dataset_original, dataset_new_perfect, tolerance=1e-6)

    # Step 5b: Record imperfect implementation
    print("\n" + "=" * 70)
    print("SCENARIO B: Implementation with Small Errors")
    print("=" * 70)
    dataset_new_error = record_new_implementation(test_inputs, introduce_error=True)

    # Step 6b: Compare imperfect
    compare_implementations(dataset_original, dataset_new_error, tolerance=1e-6)

    # Step 7: Demonstrate playback
    demonstrate_playback(dataset_original)

    # Summary
    print("\n" + "=" * 70)
    print("WORKFLOW COMPLETE")
    print("=" * 70)
    print("\nKey Takeaways:")
    print("  • DataRecorder captures component I/O for verification")
    print("  • Dataset provides backend-agnostic storage")
    print("  • DataPlayback enables replaying recorded data")
    print("  • diffdiff verifies implementation correctness")
    print("  • Workflow applies to both AI-compile and NN-compile")

    print("\nNext Steps:")
    print("  • Save datasets to disk for persistent storage")
    print("  • Integrate with actual C++ compilation pipeline")
    print("  • Add visualization of differences")
    print("  • Automate regression testing")

    print("\n" + "=" * 70 + "\n")
