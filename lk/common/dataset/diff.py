#!/usr/bin/env python3

"""
    Dataset comparison utilities for verification.
    
    Compare two datasets to verify correctness of:
    - Cross-language implementations (AI-compile)
    - Neural network approximations (NN-compile)
    - System changes (regression testing)
"""

# BAM
from lk.common.dataset.dataset import Dataset
from lk.msgs.msg import Msg

# PYTHON
from typing import Any, Optional
from dataclasses import dataclass, field
import numpy as np


@dataclass
class DiffResult:
    """
        Result of comparing two datasets.
        
        Contains detailed information about differences found.
    """
    keys_match: bool
    timestamps_match: bool
    values_match: bool
    
    missing_keys_in_b: list[str] = field(default_factory=list)
    extra_keys_in_b: list[str] = field(default_factory=list)
    
    key_diffs: dict[str, 'KeyDiff'] = field(default_factory=dict)
    
    @property
    def all_match(self) -> bool:
        """Check if datasets are identical."""
        return self.keys_match and self.timestamps_match and self.values_match
    
    def summary(self) -> str:
        """Get human-readable summary."""
        lines = []
        lines.append("="*70)
        lines.append("Dataset Comparison Summary")
        lines.append("="*70)
        
        lines.append(f"\nKeys match: {self.keys_match}")
        if self.missing_keys_in_b:
            lines.append(f"  Missing in B: {self.missing_keys_in_b}")
        if self.extra_keys_in_b:
            lines.append(f"  Extra in B: {self.extra_keys_in_b}")
        
        lines.append(f"\nTimestamps match: {self.timestamps_match}")
        lines.append(f"Values match: {self.values_match}")
        
        if not self.values_match:
            lines.append(f"\nDifferences by key:")
            for key, key_diff in self.key_diffs.items():
                lines.append(f"\n  {key}:")
                lines.append(f"    Length: A={key_diff.length_a}, B={key_diff.length_b}")
                lines.append(f"    Matching samples: {key_diff.n_matching}/{key_diff.length_a}")
                if key_diff.max_error is not None:
                    lines.append(f"    Max error: {key_diff.max_error}")
                    lines.append(f"    Mean error: {key_diff.mean_error}")
        
        lines.append("\n" + "="*70)
        return "\n".join(lines)


@dataclass
class KeyDiff:
    """
        Difference information for a single key.
    """
    key: str
    length_a: int
    length_b: int
    n_matching: int = 0
    n_different: int = 0
    
    # Numeric comparison stats
    max_error: Optional[float] = None
    mean_error: Optional[float] = None
    errors: list[float] = field(default_factory=list)
    
    # Sample-by-sample comparison
    mismatches: list[tuple[int, Any, Any]] = field(default_factory=list)  # (index, val_a, val_b)


def compare_values(val_a: Any, val_b: Any, tol: float = 1e-6) -> tuple[bool, Optional[float]]:
    """
        Compare two values with tolerance.
        
        Args:
            val_a: First value
            val_b: Second value
            tol: Tolerance for numeric comparisons
            
        Returns:
            Tuple of (match, error) where error is None for non-numeric types
    """
    # Handle None
    if val_a is None and val_b is None:
        return True, None
    if val_a is None or val_b is None:
        return False, None
    
    # Handle Msg types
    if isinstance(val_a, Msg) and isinstance(val_b, Msg):
        # Convert to dicts and compare
        dict_a = val_a.to_dict()
        dict_b = val_b.to_dict()
        return compare_values(dict_a, dict_b, tol)
    
    # Handle dicts
    if isinstance(val_a, dict) and isinstance(val_b, dict):
        if set(val_a.keys()) != set(val_b.keys()):
            return False, None
        
        for key in val_a.keys():
            match, _ = compare_values(val_a[key], val_b[key], tol)
            if not match:
                return False, None
        return True, None
    
    # Handle lists/arrays
    if isinstance(val_a, (list, tuple, np.ndarray)) and isinstance(val_b, (list, tuple, np.ndarray)):
        arr_a = np.asarray(val_a)
        arr_b = np.asarray(val_b)
        
        if arr_a.shape != arr_b.shape:
            return False, None
        
        if np.issubdtype(arr_a.dtype, np.number) and np.issubdtype(arr_b.dtype, np.number):
            diff = np.abs(arr_a - arr_b)
            max_diff = np.max(diff)
            return max_diff <= tol, float(max_diff)
        else:
            # Non-numeric arrays, compare element-wise
            return np.array_equal(arr_a, arr_b), None
    
    # Handle numeric types
    if isinstance(val_a, (int, float, np.number)) and isinstance(val_b, (int, float, np.number)):
        error = abs(float(val_a) - float(val_b))
        return error <= tol, error
    
    # Handle strings and other types
    return val_a == val_b, None


def diffdiff(dataset_a: Dataset, 
            dataset_b: Dataset,
            tol: float = 1e-6,
            check_timestamps: bool = True,
            timestamp_tol: float = 1e-3) -> DiffResult:
    """
        Compare two datasets (the "diff diff" comparison).
        
        Inspired by git's diff diff - compares two recorded datasets
        to verify that different implementations produce same results.
        
        Args:
            dataset_a: First dataset (reference/ground truth)
            dataset_b: Second dataset (new implementation)
            tol: Tolerance for value comparisons
            check_timestamps: Whether to compare timestamps
            timestamp_tol: Tolerance for timestamp comparisons
            
        Returns:
            DiffResult with detailed comparison information
            
        Example:
            >>> dataset_a = record_original_component(inputs)
            >>> dataset_b = record_new_implementation(inputs)
            >>> result = diffdiff(dataset_a, dataset_b)
            >>> if result.all_match:
            ...     print("✓ Implementation matches!")
            >>> else:
            ...     print(result.summary())
    """
    result = DiffResult(
        keys_match=False,
        timestamps_match=True,
        values_match=True
    )
    
    # Compare keys
    keys_a = set(dataset_a.keys())
    keys_b = set(dataset_b.keys())
    
    result.missing_keys_in_b = list(keys_a - keys_b)
    result.extra_keys_in_b = list(keys_b - keys_a)
    result.keys_match = (keys_a == keys_b)
    
    # Compare data for common keys
    common_keys = keys_a & keys_b
    
    for key in common_keys:
        data_a = dataset_a.read(key)
        data_b = dataset_b.read(key)
        
        key_diff = KeyDiff(
            key=key,
            length_a=len(data_a),
            length_b=len(data_b)
        )
        
        # Compare lengths
        if len(data_a) != len(data_b):
            result.values_match = False
            key_diff.n_different = abs(len(data_a) - len(data_b))
            result.key_diffs[key] = key_diff
            continue
        
        # Compare timestamps if requested
        if check_timestamps:
            ts_a = dataset_a.get_timestamps(key)
            ts_b = dataset_b.get_timestamps(key)
            
            if len(ts_a) == len(ts_b):
                ts_diff = np.abs(np.array(ts_a) - np.array(ts_b))
                if np.max(ts_diff) > timestamp_tol:
                    result.timestamps_match = False
        
        # Compare values
        errors = []
        for i, (val_a, val_b) in enumerate(zip(data_a, data_b)):
            match, error = compare_values(val_a, val_b, tol)
            
            if match:
                key_diff.n_matching += 1
            else:
                key_diff.n_different += 1
                result.values_match = False
                
                if error is not None:
                    errors.append(error)
                
                # Store first few mismatches for inspection
                if len(key_diff.mismatches) < 10:
                    key_diff.mismatches.append((i, val_a, val_b))
        
        # Calculate error statistics
        if errors:
            key_diff.errors = errors
            key_diff.max_error = max(errors)
            key_diff.mean_error = np.mean(errors)
        
        result.key_diffs[key] = key_diff
    
    return result


def assert_datasets_equal(dataset_a: Dataset,
                         dataset_b: Dataset,
                         tol: float = 1e-6,
                         message: str = "") -> None:
    """
        Assert that two datasets are equal (for testing).
        
        Args:
            dataset_a: First dataset
            dataset_b: Second dataset
            tol: Comparison tolerance
            message: Optional error message prefix
            
        Raises:
            AssertionError: If datasets don't match
    """
    result = diffdiff(dataset_a, dataset_b, tol=tol)
    
    if not result.all_match:
        error_msg = message + "\n" if message else ""
        error_msg += result.summary()
        raise AssertionError(error_msg)


if __name__ == '__main__':
    
    print("\n" + "="*70)
    print("DiffDiff - Dataset Comparison for Verification")
    print("="*70)
    
    from lk.msgs.msg import Action
    
    # Example 1: Identical datasets
    print("\nExample 1: Identical datasets")
    dataset_a = Dataset.create(backend='memory')
    dataset_b = Dataset.create(backend='memory')
    
    for i in range(5):
        action = Action(data=[i, i+1])
        dataset_a.write('action', action, timestamp=i*0.1)
        dataset_b.write('action', action, timestamp=i*0.1)
    
    result = diffdiff(dataset_a, dataset_b)
    print(f"  All match: {result.all_match}")
    print(f"  Keys match: {result.keys_match}")
    print(f"  Values match: {result.values_match}")
    
    # Example 2: Different values
    print("\nExample 2: Datasets with differences")
    dataset_c = Dataset.create(backend='memory')
    dataset_d = Dataset.create(backend='memory')
    
    for i in range(5):
        dataset_c.write('x', Action(data=[i]))
        dataset_d.write('x', Action(data=[i + 0.1]))  # Slight difference
    
    result2 = diffdiff(dataset_c, dataset_d, tol=0.05)
    print(result2.summary())
    
    # Example 3: Missing keys
    print("\nExample 3: Missing keys")
    dataset_e = Dataset.create(backend='memory')
    dataset_f = Dataset.create(backend='memory')
    
    dataset_e.write('sensor1', Action(data=[1]))
    dataset_e.write('sensor2', Action(data=[2]))
    dataset_f.write('sensor1', Action(data=[1]))
    
    result3 = diffdiff(dataset_e, dataset_f)
    print(f"  Keys match: {result3.keys_match}")
    print(f"  Missing in B: {result3.missing_keys_in_b}")
    
    # Example 4: Assertion
    print("\nExample 4: Using assert_datasets_equal")
    try:
        assert_datasets_equal(dataset_a, dataset_b)
        print("  ✓ Assertion passed - datasets are equal")
    except AssertionError as e:
        print(f"  ✗ Assertion failed: {e}")
    
    print("\n" + "="*70)
    print("DiffDiff enables verification of implementations")
    print("Use it to compare original vs compiled/approximated versions")
    print("="*70 + "\n")


