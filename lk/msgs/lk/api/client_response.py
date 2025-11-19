"""
    ClientResponse - Generic response container for API calls.
    
    ROS-free version that acts as both a success flag and data carrier.
    
    Usage Examples:
    
    # Simple success check
    response = some_api_call()
    if response:  # Equivalent to: if response.success
        print("Success!")
    
    # Access data and header
    response = some_api_call()
    if response:
        result = response.data
        print(f"Processing took {response.header.process_duration:.3f}s")
    
    # Tuple unpacking for convenience
    response, result = some_api_call()
    if response:
        use(result)
    
    # Propagate failures up call stack
    response = downstream_call()
    if not response:
        return ClientResponse.failure(None, response.header.error_msg)
"""

# BAM
from .error_code import ErrorCode
from .response_header import ResponseHeader

# PYTHON
from typing import TypeVar, Generic, Optional, Iterator

T = TypeVar('T')


class ClientResponse(Generic[T]):
    """Generic response container that acts as success flag and data carrier.
    
    Can be used as:
    - Boolean (success check)
    - Tuple (response, data) via unpacking
    - Object with header and data attributes
    """
    
    def __init__(self, header: ResponseHeader, data: Optional[T] = None, info: Optional[dict] = None):
        """Create a ClientResponse.
        
        DescriptionArgs:
            header: ResponseHeader with error code and timing info
            data: The response data (can be None)
            info: Free-form dictionary for additional information (can be None)
        """
        self.data = data
        self.header = header
        self.info = info if info is not None else {}
    
    @classmethod
    def success(cls, error_msg: str = "", data: Optional[T] = None, info: Optional[dict] = None) -> "ClientResponse[T]":
        """Create a successful response."""
        header = ResponseHeader.create_success(error_msg=error_msg)
        return cls(header, data, info)
    
    @classmethod
    def failure(cls, error_msg: str = "", fix_msg: str = "", data: Optional[T] = None, info: Optional[dict] = None) -> "ClientResponse[T]":
        """Create a failure response."""
        header = ResponseHeader.create_failure(error_msg=error_msg, fix_msg=fix_msg)
        return cls(header, data, info)
    
    @classmethod
    def custom(cls, error_code: ErrorCode, error_msg: str = "", fix_msg: str = "", data: Optional[T] = None, info: Optional[dict] = None) -> "ClientResponse[T]":
        """Create a response with custom error code."""
        header = ResponseHeader.create_custom(error_code=error_code, error_msg=error_msg, fix_msg=fix_msg)
        return cls(header, data, info)

    @classmethod
    def from_code(cls, error_code: ErrorCode, error_msg: str = "", fix_msg: str = "", data: Optional[T] = None, info: Optional[dict] = None) -> "ClientResponse[T]":
        """Create a response from an error code."""
        header = ResponseHeader.create_custom(error_code=error_code, error_msg=error_msg, fix_msg=fix_msg)
        return cls(header, data, info)
    
    @classmethod
    def combine_headers(cls, *responses: "ClientResponse") -> ResponseHeader:
        """Combine headers from multiple responses.
        
        Returns the first failure header, or first header if all succeed.
        """
        if not responses:
            return ResponseHeader.create_failure(error_msg="No responses provided")
        
        for response in responses:
            if not response.success:
                return response.header
        
        return responses[0].header
    
    @property
    def code(self) -> ErrorCode:
        """Get the error code."""
        return self.header.error_code
    
    @property
    def is_success(self) -> bool:
        """Check if response indicates success."""
        return self.header.success
    
    def __bool__(self) -> bool:
        """Allow 'if response:' to check success."""
        return self.is_success
    
    def __iter__(self) -> Iterator:
        """Allow tuple unpacking: response, data = some_call()"""
        return iter((self, self.data))
    
    def __str__(self) -> str:
        """Readable string representation."""
        code_str = self.header.error_code.to_str()
        msg_str = f", msg='{self.header.error_msg}'" if self.header.error_msg else ""
        fix_str = f", fix='{self.header.fix_msg}'" if self.header.fix_msg else ""
        duration_str = f", duration={self.header.process_duration:.3f}s" if self.header.process_duration > 0 else ""
        
        return f"<ClientResponse success={self.success}, code={code_str}{msg_str}{fix_str}{duration_str}>"
    
    def __repr__(self) -> str:
        """Detailed string representation."""
        code_str = self.header.error_code.to_str()
        info_str = f"  info={self.info}\n" if self.info else ""
        return (
            f"<ClientResponse\n"
            f"  success={self.success}\n"
            f"  code={code_str}\n"
            f"  error_msg='{self.header.error_msg}'\n"
            f"  fix_msg='{self.header.fix_msg}'\n"
            f"  transport={self.header.transport_duration:.3f}s\n"
            f"  process={self.header.process_duration:.3f}s\n"
            f"  data={self.data}\n"
            f"{info_str}"
            f">"
        )
    
    def inspect(self) -> None:
        """Print detailed representation of the response."""
        print(self.__repr__())

