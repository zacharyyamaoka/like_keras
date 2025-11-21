"""
ResponseHeader for API responses.

ROS-free version using Python dataclasses.
Tracks timing information and error codes.
"""

# BAM
from .error_code import ErrorCode

# PYTHON
from dataclasses import dataclass, field
from typing import Optional
import time


@dataclass
class ResponseHeader:
    """Header for API responses containing timing, status, and error information.

    Timing breakdown:
    - request_stamp: When request was created/sent
    - receive_stamp: When request was received by server
    - response_stamp: When response was sent
    - transport_duration: receive_stamp - request_stamp (network time)
    - process_duration: response_stamp - receive_stamp (processing time)
    """

    # Error information
    error_code: ErrorCode = ErrorCode.UNDEFINED
    error_msg: str = ""
    fix_msg: str = ""  # Suggested fix or solution

    # Timing information
    request_stamp: float = 0.0  # Copied from request header
    receive_stamp: float = 0.0  # When server received request
    response_stamp: float = field(default_factory=time.time)  # When response created

    # Computed durations (in seconds)
    transport_duration: float = 0.0  # Time to transmit request
    process_duration: float = 0.0  # Time to process request

    # State flags
    calibrated: bool = False  # Is the system calibrated?

    @classmethod
    def create_success(cls, error_msg: str = "", **kwargs) -> "ResponseHeader":
        """Create a successful response header."""
        return cls(error_code=ErrorCode.SUCCESS, error_msg=error_msg, **kwargs)

    @classmethod
    def create_failure(cls, error_msg: str = "", **kwargs) -> "ResponseHeader":
        """Create a failure response header."""
        return cls(error_code=ErrorCode.FAILURE, error_msg=error_msg, **kwargs)

    @classmethod
    def create_custom(
        cls, error_code: ErrorCode, error_msg: str = "", **kwargs
    ) -> "ResponseHeader":
        """Create a response header with custom error code."""
        return cls(error_code=error_code, error_msg=error_msg, **kwargs)

    @classmethod
    def from_request(
        cls,
        request_header,
        error_code: ErrorCode = ErrorCode.SUCCESS,
        error_msg: str = "",
    ) -> "ResponseHeader":
        """Create response header from request header with timing info.

        DescriptionArgs:
            request_header: RequestHeader or object with 'stamp' attribute
            error_code: Response error code
            error_msg: Response error message
        """
        receive_stamp = time.time()
        request_stamp = getattr(request_header, "stamp", 0.0)

        transport_duration = receive_stamp - request_stamp if request_stamp > 0 else 0.0

        return cls(
            error_code=error_code,
            error_msg=error_msg,
            request_stamp=request_stamp,
            receive_stamp=receive_stamp,
            response_stamp=receive_stamp,  # Will be updated when response is sent
            transport_duration=transport_duration,
            process_duration=0.0,  # Will be computed when processing completes
        )

    def finalize(self):
        """Finalize response header by computing final durations.

        Call this right before sending the response.
        """
        self.response_stamp = time.time()
        if self.receive_stamp > 0:
            self.process_duration = self.response_stamp - self.receive_stamp

    @property
    def success(self) -> bool:
        """Check if response indicates success."""
        return self.error_code == ErrorCode.SUCCESS

    @property
    def total_duration(self) -> float:
        """Total time from request to response."""
        return self.transport_duration + self.process_duration
