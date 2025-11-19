
from contextlib import contextmanager
import os
import tempfile


"""
Suffix is important as some systems check it.

"""
# Xacro doc doesn't have temp srdf, so implement function here
@contextmanager
def temp_srdf_file(srdf: str):
    fd, path = tempfile.mkstemp(suffix=".srdf")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(srdf)
        yield path
    finally:
        os.remove(path)

@contextmanager
def temp_urdf_file(urdf: str):
    fd, path = tempfile.mkstemp(suffix=".urdf")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(urdf)
        yield path
    finally:
        os.remove(path)

@contextmanager
def temp_xacro_file(xacro: str):
    fd, path = tempfile.mkstemp(suffix=".xacro")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(xacro)
        yield path
    finally:
        os.remove(path)