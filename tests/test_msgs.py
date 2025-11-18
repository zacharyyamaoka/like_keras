#!/usr/bin/env python3

"""
    Tests for message types.
"""

# BAM
from lk.msgs.msg import Msg, Observation, Action, Reward, Done, Info

# PYTHON
import numpy as np
import pytest
from typing import Optional


def test_msg_base():
    """Test Msg base class."""
    msg = Msg()
    assert msg is not None
    
    # Test serialization
    data = msg.to_dict()
    assert isinstance(data, dict)
    
    # Test deserialization
    msg2 = Msg.from_dict(data)
    assert msg2 is not None


def test_observation():
    """Test Observation message."""
    data = np.array([1.0, 2.0, 3.0])
    obs = Observation(data=data)
    
    assert obs.data is not None
    assert obs.shape == data.shape
    assert obs.dtype == str(data.dtype)
    
    # Test with explicit shape/dtype
    obs2 = Observation(data=[1, 2, 3], shape=(3,), dtype='int')
    assert obs2.shape == (3,)
    assert obs2.dtype == 'int'


def test_action():
    """Test Action message."""
    action = Action(data=0.5, action_space='continuous')
    
    assert action.data == 0.5
    assert action.action_space == 'continuous'
    
    # Test discrete action
    action2 = Action(data=2, action_space='discrete')
    assert action2.data == 2


def test_reward():
    """Test Reward message."""
    reward = Reward(value=1.5)
    
    assert reward.value == 1.5


def test_done():
    """Test Done message."""
    done = Done(value=True)
    
    assert done.value is True
    assert done.truncated is False
    
    # Test with truncated
    done2 = Done(value=True, truncated=True)
    assert done2.truncated is True


def test_info():
    """Test Info message."""
    info = Info(data={'score': 100, 'level': 2})
    
    assert info['score'] == 100
    assert info['level'] == 2
    assert info.get('missing', 'default') == 'default'
    
    # Test dict-like operations
    info['new_key'] = 'new_value'
    assert info['new_key'] == 'new_value'


def test_nested_dataclasses():
    """Test nested dataclass serialization with dacite."""
    from dataclasses import dataclass
    
    @dataclass
    class NestedMsg(Msg):
        value: int
        name: str
    
    @dataclass
    class ParentMsg(Msg):
        nested: NestedMsg
        count: int
        optional_nested: Optional[NestedMsg] = None
    
    # Test with nested dataclass
    nested = NestedMsg(value=42, name='test')
    msg = ParentMsg(nested=nested, count=10)
    
    # Serialize to dict
    msg_dict = msg.to_dict()
    assert msg_dict == {
        'nested': {'value': 42, 'name': 'test'},
        'count': 10,
        'optional_nested': None
    }
    
    # Deserialize from dict (this is where dacite shines)
    restored = ParentMsg.from_dict({
        'nested': {'value': 42, 'name': 'test'},
        'count': 10
    })
    
    assert restored.nested.value == 42
    assert restored.nested.name == 'test'
    assert restored.count == 10
    assert restored.optional_nested is None
    
    # Test with optional nested field
    msg_with_optional = ParentMsg.from_dict({
        'nested': {'value': 1, 'name': 'a'},
        'count': 5,
        'optional_nested': {'value': 2, 'name': 'b'}
    })
    
    assert msg_with_optional.optional_nested is not None
    assert msg_with_optional.optional_nested.value == 2


if __name__ == "__main__":
    pytest.main([__file__, '-v'])

