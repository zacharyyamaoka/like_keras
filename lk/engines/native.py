#!/usr/bin/env python3

"""
    Native Python execution engine.
    
    Executes systems in-process using pure Python.
    Best for development, debugging, and single-machine prototyping.
"""

# BAM
from lk.engines.engine import Engine

# PYTHON
from pathlib import Path
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from lk.common.system import System


class NativeEngine(Engine):
    """
    Native Python execution engine.
    
    Runs systems directly in-process without any compilation step.
    Uses the current implementation of System.run() for execution.
    
    Advantages:
    - Fast iteration (no compilation step)
    - Easy debugging (single process)
    - Simple deployment (no external dependencies)
    
    Use for:
    - Development and testing
    - Single-machine prototypes
    - Simple applications
    """
    
    def compile(self, system: "System") -> Optional[Path]:
        """
        No compilation needed for native execution.
        
        Args:
            system: System to compile
            
        Returns:
            None (no config file generated)
        """
        # Validate system
        is_valid, issues = self.validate(system)
        if not is_valid:
            raise ValueError(f"System validation failed:\n" + "\n".join(issues))
        
        # No compilation needed
        return None
    
    def launch(self, system: "System", config_path: Optional[Path] = None, **kwargs):
        """
        Launch system in-process.
        
        Uses the system's built-in lifecycle methods:
        configure() -> activate() -> run() -> deactivate() -> shutdown()
        
        Args:
            system: System to launch
            config_path: Ignored for native engine
            **kwargs: Additional launch options
                - iterations: Number of iterations to run
        """
        iterations = kwargs.get('iterations', None)
        
        try:
            # Standard lifecycle
            system.configure()
            system.activate()
            system.run(iterations)
        except KeyboardInterrupt:
            if system.config.verbose:
                print("\nSystem interrupted by user")
        except Exception as e:
            if system.config.verbose:
                print(f"Error during execution: {e}")
            raise
        finally:
            # Always cleanup
            system.deactivate()
            system.shutdown()
    
    def validate(self, system: "System") -> tuple[bool, list[str]]:
        """
        Validate system for native execution.
        
        Native engine is very permissive - most systems should work.
        
        Args:
            system: System to validate
            
        Returns:
            Tuple of (is_valid, list_of_issues)
        """
        issues = []
        
        # Check for distributed nodes
        for node in system.nodes:
            if node.machine != "local":
                issues.append(
                    f"Node '{node.name}' assigned to machine '{node.machine}'. "
                    "Native engine only supports local execution. "
                    "Use engine='dora' or 'ros2' for distributed systems."
                )
        
        # Check for mixed engines
        engines = {node.engine for node in system.nodes if node.engine != 'native'}
        if engines:
            issues.append(
                f"Some nodes use non-native engines: {engines}. "
                "All nodes must use engine='native' for native execution."
            )
        
        is_valid = len(issues) == 0
        return is_valid, issues

