"""
Agent implementations and training loops.
"""

# BAM
# Auto-register built-in agents when module is imported
from lk.agent import random_agent  # This triggers random_agent auto-registration
from lk.agent.agent import Agent

__all__ = ["Agent", "RandomAgent"]

# Re-export for convenience
RandomAgent = random_agent.RandomAgent
