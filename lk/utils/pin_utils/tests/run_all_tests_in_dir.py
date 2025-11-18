#!/usr/bin/env python3

from pathlib import Path
import pytest


if __name__ == "__main__":
    pytest.main([str(Path(__file__).resolve().parent)])
