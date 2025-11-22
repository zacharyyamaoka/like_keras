#!/usr/bin/env python3

"""
    Backends package - Compilation targets for like-keras systems
"""

# Export backend classes for convenience
from lk.common.backend import Backend, BackendType, BackendConfig, create_backend

__all__ = ['Backend', 'BackendType', 'BackendConfig', 'create_backend']

