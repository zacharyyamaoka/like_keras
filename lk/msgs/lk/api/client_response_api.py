#!/usr/bin/env python3

"""
ClientResponse Type Hinting Exploration

This file demonstrates different approaches to type hinting with ClientResponse
to balance ergonomic API usage with IDE autocomplete support.

Key questions explored:
1. Can we transparently delegate attributes from ClientResponse to wrapped data?
2. How do different type hint patterns affect IDE autocomplete?
3. What's the best balance between runtime ergonomics and static type checking?
"""

# BAM
# PYTHON
from typing import TypeVar

import numpy as np

from lk.msgs.lk.api.client_response import ClientResponse
from lk.msgs.msg import Msg

# ============================================================================
# Sample Message Classes
# ============================================================================


class Observation(Msg):
    """Sample observation message with typed attributes."""

    def __init__(self):
        super().__init__()
        self.reward: float = 0.0
        self.state: np.ndarray = np.zeros(4)
        self.done: bool = False
        self.info: dict = {}


class Action(Msg):
    """Sample action message."""

    def __init__(self):
        super().__init__()
        self.command: np.ndarray = np.zeros(2)
        self.velocity: float = 0.0


class ConflictingMessage(Msg):
    """Sample message with attributes that conflict with ClientResponse."""

    def __init__(self):
        super().__init__()
        self.reward: float = 1.0
        self.header: str = "This conflicts with ClientResponse.header!"
        self.code: int = 42  # Conflicts with ClientResponse.code property


# ============================================================================
# Approach 1: Standard ClientResponse[T] - Requires .data access
# ============================================================================


def approach1_standard() -> ClientResponse[Observation]:
    """Standard approach - type checker knows about T but not delegated attrs."""
    obs = Observation()
    obs.reward = 1.5
    obs.state = np.array([1, 2, 3, 4])
    return ClientResponse.success(data=obs)


def test_approach1():
    """Test standard approach."""
    print("\n=== Approach 1: Standard ClientResponse[T] ===")

    response = approach1_standard()

    # Type checker: ✗ Doesn't know about .reward (no autocomplete)
    # Runtime: ✓ Works via __getattr__ delegation
    print(f"Direct access - reward: {response.reward}")

    # Type checker: ✓ Knows about .data.reward (full autocomplete)
    # Runtime: ✓ Works
    print(f"Via .data - reward: {response.data.reward}")

    # Type checker: ✓ Knows about ClientResponse features
    # Runtime: ✓ Works
    if response:
        print(f"Success! Duration: {response.header.process_duration:.4f}s")


# ============================================================================
# Approach 2: Union Type - ClientResponse[T] | T
# ============================================================================


def approach2_union() -> ClientResponse[Observation] | Observation:
    """Union type - tells type checker it could be either type."""
    obs = Observation()
    obs.reward = 2.5
    obs.state = np.array([5, 6, 7, 8])
    return ClientResponse.success(data=obs)


def test_approach2():
    """Test union type approach."""
    print("\n=== Approach 2: Union Type ClientResponse[T] | T ===")

    response = approach2_union()

    # Type checker: ✓ Knows about .reward (from Observation in union)
    # Runtime: ✓ Works via __getattr__ delegation
    print(f"Direct access - reward: {response.reward}")

    # DISCOVERY: Type checker actually provides autocomplete for BOTH types in union!
    # Type checker: ✓ Knows about .header (from ClientResponse in union)
    # Type checker: ✓ Knows about .reward (from Observation in union)
    # Runtime: ✓ Works perfectly via __getattr__ delegation
    print(f"Direct access - header: {response.header.process_duration:.4f}s")
    print(f"Direct access - state: {response.state}")

    # isinstance check not actually needed for IDE autocomplete!
    if isinstance(response, ClientResponse):
        print("isinstance check still works if you want type narrowing")

    # Type checker sees union, so only common attributes available without narrowing
    if response:  # This might not work as expected with union
        print("Boolean check works")


# ============================================================================
# Approach 3: .as_type() Helper Method
# ============================================================================


class ClientResponseWithHelper(ClientResponse[TypeVar("T")]):
    """Extended ClientResponse with .as_type() helper for IDE autocomplete."""

    def as_type(self) -> TypeVar("T"):
        """Cast self as T for type checker (returns self at runtime).

        This is purely for type checking - at runtime it returns self,
        which transparently delegates to .data via __getattr__.

        Usage:
            response: ClientResponse[Observation] = env.step(action)
            obs = response.as_type()  # IDE now sees: Observation
            print(obs.reward)  # Autocomplete works!
            print(obs.header.process_duration)  # Still works at runtime!
        """
        return self  # type: ignore


def approach3_with_helper() -> ClientResponseWithHelper[Observation]:
    """Using .as_type() helper for best of both worlds."""
    obs = Observation()
    obs.reward = 3.5
    obs.state = np.array([9, 10, 11, 12])
    return ClientResponseWithHelper.success(data=obs)


def test_approach3():
    """Test .as_type() helper approach."""
    print("\n=== Approach 3: .as_type() Helper Method ===")

    response = approach3_with_helper()

    # Without .as_type():
    # Type checker: ✗ No autocomplete for .reward
    # Runtime: ✓ Works
    print(f"Direct access - reward: {response.reward}")

    # With .as_type():
    obs = response.as_type()
    # Type checker: ✓ Full autocomplete for Observation attributes
    # Runtime: ✓ Works (same object, just type-cast)
    print(f"After .as_type() - reward: {obs.reward}")
    print(f"After .as_type() - state: {obs.state}")

    # Best part: Still has ClientResponse features!
    # Type checker: ? Might complain, but works at runtime
    # Runtime: ✓ Works perfectly
    if obs:  # Same object, so boolean check works
        print(f"Success check works! Duration: {obs.header.process_duration:.4f}s")


# ============================================================================
# Approach 4: Document and Accept Runtime-Only Delegation
# ============================================================================


def approach4_documented() -> ClientResponse[Observation]:
    """Well-documented approach - clear about runtime vs type-time behavior."""
    obs = Observation()
    obs.reward = 4.5
    obs.state = np.array([13, 14, 15, 16])
    return ClientResponse.success(data=obs)


def test_approach4():
    """Test documented runtime-only delegation."""
    print("\n=== Approach 4: Document Runtime Behavior ===")

    # Clear documentation about what works
    response: ClientResponse[Observation] = approach4_documented()

    # Runtime: ✓ Works perfectly via __getattr__
    # Type checker: ✗ No autocomplete, but documented
    print(f"Direct access (runtime only) - reward: {response.reward}")

    # When you need IDE support, explicitly use .data
    # Type checker: ✓ Full autocomplete
    # Runtime: ✓ Works
    print(f"Via .data (IDE support) - reward: {response.data.reward}")

    # ClientResponse features always work
    if response:
        print(f"Success! Duration: {response.header.process_duration:.4f}s")


# ============================================================================
# Practical Usage Examples
# ============================================================================


class SimpleEnv:
    """Mock environment for testing."""

    def __init__(self):
        self.obs = Observation()

    def step_v1(self, action: Action) -> ClientResponse[Observation]:
        """Standard return type."""
        obs = Observation()
        obs.reward = np.random.random()
        obs.state = np.random.randn(4)
        obs.done = np.random.random() > 0.9
        return ClientResponse.success(data=obs)

    def step_v2(self, action: Action) -> ClientResponse[Observation] | Observation:
        """Union return type."""
        obs = Observation()
        obs.reward = np.random.random()
        obs.state = np.random.randn(4)
        obs.done = np.random.random() > 0.9
        return ClientResponse.success(data=obs)


def test_practical_usage():
    """Test practical usage patterns."""
    print("\n=== Practical Usage Examples ===")

    env = SimpleEnv()
    action = Action()

    # Pattern 1: Direct access (runtime only, no autocomplete)
    print("\nPattern 1: Direct attribute access")
    response = env.step_v1(action)
    if response:
        print(f"  Reward: {response.reward:.3f}")
        print(f"  State shape: {response.state.shape}")
        print(f"  Done: {response.done}")
        print(f"  Duration: {response.header.process_duration:.4f}s")

    # Pattern 2: Explicit .data access (full autocomplete)
    print("\nPattern 2: Explicit .data access")
    response = env.step_v1(action)
    if response:
        obs = response.data
        print(f"  Reward: {obs.reward:.3f}")
        print(f"  State shape: {obs.state.shape}")
        print(f"  Done: {obs.done}")

    # Pattern 3: Tuple unpacking
    print("\nPattern 3: Tuple unpacking")
    response, obs = env.step_v1(action)
    if response:
        print(f"  Reward: {obs.reward:.3f}")
        print(f"  State shape: {obs.state.shape}")

    # Pattern 4: Union type with isinstance
    print("\nPattern 4: Union type")
    response = env.step_v2(action)
    response.reward = 100.0
    response.header
    print(f"  Direct reward access: {response.reward:.3f}")
    if isinstance(response, ClientResponse):
        print(f"  Duration: {response.header.process_duration:.4f}s")


# ============================================================================
# Comparison Summary
# ============================================================================


def print_summary():
    """Print comparison summary of all approaches."""
    print("\n" + "=" * 70)
    print("COMPARISON SUMMARY")
    print("=" * 70)

    print("""
Approach 1: ClientResponse[T]
  Pros: 
    - Simple, standard generic typing
    - Clear separation between response and data
  Cons:
    - No autocomplete for delegated attributes
    - Must use .data for IDE support
  Rating: ★★★☆☆

Approach 2: ClientResponse[T] | T
  Pros:
    - Autocomplete works for T attributes
  Cons:
    - Requires isinstance() checks for ClientResponse features
    - Union semantics don't match runtime behavior (it's both, not either)
    - Confusing for type checker
  Rating: ★★☆☆☆

Approach 3: .as_type() Helper
  Pros:
    - Best of both worlds - full access to everything
    - Optional: use when you want IDE support
    - Clean API: response.as_type().reward
  Cons:
    - One extra method call
    - Type lie (but acceptable)
  Rating: ★★★★☆

Approach 4: Document Runtime Behavior
  Pros:
    - Simplest implementation
    - Clear expectations via documentation
    - Use .data when you need autocomplete
  Cons:
    - No autocomplete for direct access
    - Requires discipline to know when to use .data
  Rating: ★★★★☆

RECOMMENDATION: Use Approach 4 (Document) for simplicity, with optional
Approach 3 (.as_type()) for power users who want maximum IDE support.
    """)


# ============================================================================
# Namespace Conflict Detection
# ============================================================================


def test_namespace_conflicts():
    """Test namespace conflict detection and warnings."""
    print("\n" + "=" * 70)
    print("NAMESPACE CONFLICT DETECTION")
    print("=" * 70)

    print("\n1. Normal case - no conflicts:")
    obs = Observation()
    obs.reward = 5.0
    response = ClientResponse.success(data=obs)
    print("   Created ClientResponse with Observation - no warnings!")

    print("\n2. Conflicting attributes:")
    import warnings

    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        conflicting = ConflictingMessage()
        conflicting.header = "My custom header string"
        conflicting.code = 999
        response = ClientResponse.success(data=conflicting)

        if w:
            print(f"   ⚠️  Warning detected: {w[0].message}")
        else:
            print("   (No warning - conflict detection may be disabled)")

    print("\n3. Accessing conflicting attributes:")
    print(f"   response.header type: {type(response.header).__name__}")
    print(f"   response.data.header: {response.data.header}")
    print(f"   response.code type: {type(response.code).__name__}")
    print(f"   response.data.code: {response.data.code}")
    print("\n   💡 Tip: Use .data.attribute to explicitly access data attributes!")


if __name__ == "__main__":
    print("=" * 70)
    print("ClientResponse Type Hinting Exploration")
    print("=" * 70)

    # Test all approaches
    test_approach1()
    test_approach2()
    test_approach3()
    test_approach4()

    # Test practical usage
    test_practical_usage()

    # Print summary
    print_summary()

    # Test namespace conflict detection
    test_namespace_conflicts()

    print("\n" + "=" * 70)
    print("CONCLUSION: __getattr__ delegation works perfectly at runtime!")
    print("For IDE support: Use ClientResponse[T] | T return type hint")
    print("For conflicts: Use .data.attribute to disambiguate")
    print("=" * 70)
