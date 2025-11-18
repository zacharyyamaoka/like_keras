"""
    Message definitions and types
    
    All data passed through ports must inherit from Msg.
"""

# BAM
from lk.msgs.msg import Msg, Observation, Action, Reward, Done, Info

__all__ = [
    'Msg',
    'Observation',
    'Action',
    'Reward',
    'Done',
    'Info',
]

