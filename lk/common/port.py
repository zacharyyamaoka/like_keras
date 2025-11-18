#!/usr/bin/env python3

"""
    Port system for component connections.
    
    Ports enable type-safe data flow between components.
    All port data must be Msg subclasses.
"""

# BAM
from lk.msgs.msg import Msg

# PYTHON
from typing import TypeVar, Generic, Optional, List, Type, get_args, get_origin
from dataclasses import dataclass, field


T = TypeVar('T', bound=Msg)


class Port(Generic[T]):
    """
        Generic port for component communication.
        
        Ports hold typed data and track connections.
        Type parameter T must be a Msg subclass.
    """
    
    def __init__(self, name: str, msg_type: Type[T], owner: Optional['Component'] = None):
        """
            Initialize port.
            
            Args:
                name: Port identifier
                msg_type: Message type (must inherit from Msg)
                owner: Component that owns this port
        """
        self.name = name
        self.msg_type = msg_type
        self.owner = owner
        self._value: Optional[T] = None
        self._connections: List['Connection'] = []
        
        # Validate that msg_type is a Msg subclass
        if not (isinstance(msg_type, type) and issubclass(msg_type, Msg)):
            raise TypeError(f"Port type must be a Msg subclass, got {msg_type}")
    
    @property
    def value(self) -> Optional[T]:
        """Get current value in port."""
        return self._value
    
    @value.setter
    def value(self, val: T):
        """
            Set port value with type validation.
            
            Args:
                val: New value (must match port's msg_type)
        """
        if val is not None and not isinstance(val, self.msg_type):
            raise TypeError(
                f"Port {self.name} expects {self.msg_type.__name__}, "
                f"got {type(val).__name__}"
            )
        self._value = val
    
    def connect(self, connection: 'Connection'):
        """Register a connection to this port."""
        if connection not in self._connections:
            self._connections.append(connection)
    
    def disconnect(self, connection: 'Connection'):
        """Remove a connection from this port."""
        if connection in self._connections:
            self._connections.remove(connection)
    
    @property
    def connections(self) -> List['Connection']:
        """Get all connections to this port."""
        return self._connections.copy()
    
    def __repr__(self) -> str:
        owner_name = self.owner.__class__.__name__ if self.owner else "None"
        return f"Port(name={self.name}, type={self.msg_type.__name__}, owner={owner_name})"


class InputPort(Port[T]):
    """
        Input port for receiving data.
        
        Data flows INTO the component through input ports.
    """
    
    def read(self) -> Optional[T]:
        """Read current value from port."""
        return self.value
    
    def __repr__(self) -> str:
        owner_name = self.owner.__class__.__name__ if self.owner else "None"
        return f"InputPort(name={self.name}, type={self.msg_type.__name__}, owner={owner_name})"


class OutputPort(Port[T]):
    """
        Output port for sending data.
        
        Data flows OUT of the component through output ports.
    """
    
    def write(self, val: T):
        """Write value to port and propagate to connections."""
        self.value = val
        # In the future, this could trigger async propagation
    
    def __repr__(self) -> str:
        owner_name = self.owner.__class__.__name__ if self.owner else "None"
        return f"OutputPort(name={self.name}, type={self.msg_type.__name__}, owner={owner_name})"


class PortCollection:
    """
        Container for organizing related ports.
        
        Enables access like: component.inputs.obs or component.outputs.action
    """
    
    def __init__(self, owner: Optional['Component'] = None):
        """
            Initialize port collection.
            
            Args:
                owner: Component that owns these ports
        """
        self.owner = owner
        self._ports: dict[str, Port] = {}
    
    def add_port(self, name: str, port: Port):
        """
            Add a port to the collection.
            
            Args:
                name: Port name (attribute name for access)
                port: Port instance
        """
        self._ports[name] = port
        setattr(self, name, port)
    
    def get_port(self, name: str) -> Optional[Port]:
        """Get port by name."""
        return self._ports.get(name)
    
    def all_ports(self) -> List[Port]:
        """Get all ports in collection."""
        return list(self._ports.values())
    
    def __iter__(self):
        """Iterate over ports."""
        return iter(self._ports.values())
    
    def __len__(self) -> int:
        """Number of ports in collection."""
        return len(self._ports)
    
    def __repr__(self) -> str:
        port_names = list(self._ports.keys())
        return f"PortCollection({port_names})"


# Import at end to avoid circular imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from lk.common.component import Component
    from lk.common.graph import Connection

