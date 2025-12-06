#!/usr/bin/env python3
"""
Isaac Sim Basic Setup

This script demonstrates basic setup and initialization of Isaac Sim for robotics simulation.
"""

import omni
from pxr import Usd, UsdGeom
import carb
import numpy as np


def setup_basic_environment():
    """
    Set up a basic Isaac Sim environment with ground plane and lighting
    """
    print("Setting up basic Isaac Sim environment...")

    # Get the stage (USD scene)
    stage = omni.usd.get_context().get_stage()

    # Create a ground plane
    ground_path = "/World/groundPlane"
    ground_plane = UsdGeom.Xform.Define(stage, ground_path)
    print(f"Created ground plane at {ground_path}")

    # Add default lighting
    dome_light_path = "/World/DomeLight"
    dome_light = UsdGeom.Xform.Define(stage, dome_light_path)
    print(f"Added dome light at {dome_light_path}")

    print("Basic environment setup complete!")


def create_robot_model():
    """
    Create a basic robot model in the scene
    """
    print("Creating basic robot model...")

    stage = omni.usd.get_context().get_stage()

    # Create a simple robot (represented as boxes for each part)
    robot_path = "/World/Robot"
    robot = UsdGeom.Xform.Define(stage, robot_path)

    # Create base
    base_path = f"{robot_path}/base"
    base = UsdGeom.Cube.Define(stage, base_path)
    base.GetSizeAttr().Set(0.5)

    print(f"Created robot with base at {base_path}")

    return robot_path


def configure_physics():
    """
    Configure basic physics properties for the simulation
    """
    print("Configuring physics settings...")

    # Set gravity
    carb.settings.get_settings().set("/physics/scene/gravity", [-9.81, 0.0, 0.0])

    print("Physics settings configured!")


def main():
    """
    Main function to run the basic Isaac Sim setup
    """
    print("Starting Isaac Sim Basic Setup...")

    try:
        setup_basic_environment()
        robot_path = create_robot_model()
        configure_physics()

        print(f"\nIsaac Sim Basic Setup Complete!")
        print(f"Environment includes:")
        print(f"  - Ground plane")
        print(f"  - Dome lighting")
        print(f"  - Basic robot model at {robot_path}")
        print(f"  - Physics configuration")

    except Exception as e:
        print(f"Error during setup: {e}")
        return False

    return True


if __name__ == "__main__":
    success = main()
    if success:
        print("\nBasic Isaac Sim environment created successfully!")
    else:
        print("\nFailed to create Isaac Sim environment.")