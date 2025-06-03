#!/usr/bin/env python3
"""
Traditional script entry point for the SO100 teleoperation system.
Run with: python teleop_main.py [options]
"""

import sys
import os

# Add the current directory to Python path so we can import teleop
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from teleop.main import main
import asyncio

if __name__ == "__main__":
    asyncio.run(main()) 