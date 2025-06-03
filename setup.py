#!/usr/bin/env python3
"""
Setup script for the SO100 teleoperation package.
Install with: pip install -e .
"""

from setuptools import setup, find_packages

setup(
    name="teleop",
    version="0.5.0",
    description="Teleoperation Control System for the SO100 Robot Arm(s)",
    author="Emil Rofors",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "numpy",
        "torch", 
        "scipy",
        "pybullet",
        "websockets",
        "pynput",
        "lerobot",
    ],
    entry_points={
        "console_scripts": [
            "teleop=teleop.main:main_cli",
        ],
    },
    include_package_data=True,
    zip_safe=False,
) 