#!/usr/bin/env python3

"""
Base message types for the Like Keras framework.

All data passed through ports must inherit from Msg.
This provides type safety, serialization, and validation.
"""

# PYTHON
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, Optional
import numpy as np
import dacite


@dataclass
class Msg:
    """
    Base class for all messages passed through ports.

    All port data must inherit from this class to ensure
    type safety and enable serialization/validation.

    Uses dacite for robust nested dataclass handling.
    """

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert message to dictionary for serialization.

        Handles nested dataclasses automatically.
        """
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Msg":
        """
        Create message from dictionary.

        Uses dacite to handle nested dataclasses, type conversion,
        and optional fields automatically.

        Example:
            @dataclass
            class NestedMsg(Msg):
                value: int

            @dataclass
            class ParentMsg(Msg):
                nested: NestedMsg
                name: str

            msg = ParentMsg.from_dict({
                'nested': {'value': 42},
                'name': 'test'
            })
        """
        return dacite.from_dict(data_class=cls, data=data)


@dataclass
class Observation(Msg):
    """
    Standard observation message for environments.

    Can hold various types of observation data with metadata.
    """

    data: Any
    shape: Optional[tuple] = None
    dtype: Optional[str] = None

    def __post_init__(self):
        """Infer shape and dtype if not provided."""
        if isinstance(self.data, np.ndarray):
            if self.shape is None:
                self.shape = self.data.shape
            if self.dtype is None:
                self.dtype = str(self.data.dtype)


@dataclass
class Action(Msg):
    """
    Standard action message for agents.

    Can hold various types of action data (discrete, continuous, etc.).
    """

    data: Any
    action_space: Optional[str] = (
        None  # 'discrete', 'continuous', 'multi_discrete', etc.
    )


@dataclass
class Reward(Msg):
    """
    Reward signal from environment.
    """

    value: float


@dataclass
class Done(Msg):
    """
    Episode termination signal.
    """

    value: bool
    truncated: bool = False  # Distinguish between terminal and truncated episodes


@dataclass
class Info(Msg):
    """
    Additional information from environment.

    Flexible container for debug info, metrics, etc.
    """

    data: Dict[str, Any] = field(default_factory=dict)

    def __getitem__(self, key: str) -> Any:
        """Allow dict-like access."""
        return self.data[key]

    def __setitem__(self, key: str, value: Any):
        """Allow dict-like assignment."""
        self.data[key] = value

    def get(self, key: str, default: Any = None) -> Any:
        """Safe dict-like access with default."""
        return self.data.get(key, default)
