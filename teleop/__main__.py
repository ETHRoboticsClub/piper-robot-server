"""
Entry point for running the teleoperation package as a module.
Usage: python -m teleop [options]
"""

import asyncio
from .main import main

if __name__ == "__main__":
    asyncio.run(main()) 