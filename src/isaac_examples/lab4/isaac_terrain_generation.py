#!/usr/bin/env python3
"""
Isaac Sim Terrain Generation

This script demonstrates terrain generation and environment creation in Isaac Sim.
"""

import omni
from pxr import Gf, UsdGeom, UsdPhysics
import carb
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_primitive
from omni.isaac.core.utils.nucleus import get_assets_root_path


def create_flat_terrain(world):
    """
    Create a flat terrain for basic navigation
    """
    print("Creating flat terrain...")

    # Create a large plane as the ground
    terrain_path = "/World/groundPlane"
    plane = UsdGeom.Mesh.Define(world.scene.stage, terrain_path)

    # Set plane properties
    plane.GetPointsAttr().Set([
        (-10, -10, 0), (10, -10, 0), (10, 10, 0), (-10, 10, 0)
    ])
    plane.GetFaceVertexIndicesAttr().Set([0, 1, 2, 0, 2, 3])
    plane.GetFaceVertexCountsAttr().Set([3, 3])

    print(f"Created flat terrain at {terrain_path}")

    return terrain_path


def create_rough_terrain(world):
    """
    Create a rough terrain with obstacles
    """
    print("Creating rough terrain with obstacles...")

    obstacles = []

    # Add some random boxes as obstacles
    for i in range(5):
        obstacle_path = f"/World/obstacle_{i}"
        box = create_primitive(
            prim_path=obstacle_path,
            primitive_props={"size": 0.3, "position": (np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0.15)}
        )
        obstacles.append(obstacle_path)

    # Add some ramps
    for i in range(2):
        ramp_path = f"/World/ramp_{i}"
        ramp = create_primitive(
            prim_path=ramp_path,
            primitive_type="Cone",
            primitive_props={
                "radius": 1.0,
                "height": 0.5,
                "position": (np.random.uniform(-8, 8), np.random.uniform(-8, 8), 0.25),
                "orientation": (0, 0, 0, 1)
            }
        )
        obstacles.append(ramp_path)

    print(f"Created {len(obstacles)} obstacles for rough terrain")

    return obstacles


def create_indoor_environment(world):
    """
    Create an indoor environment with rooms and furniture
    """
    print("Creating indoor environment...")

    # Create walls for a simple room
    room_size = 8
    wall_height = 3
    wall_thickness = 0.2

    walls = []
    # Create 4 walls
    wall_positions = [
        (0, -room_size/2, wall_height/2),  # South wall
        (0, room_size/2, wall_height/2),   # North wall
        (-room_size/2, 0, wall_height/2),  # West wall
        (room_size/2, 0, wall_height/2),   # East wall
    ]

    for i, pos in enumerate(wall_positions):
        wall_path = f"/World/room_wall_{i}"
        if i < 2:  # Horizontal walls (length along X)
            wall_size = (room_size, wall_thickness, wall_height)
        else:  # Vertical walls (length along Y)
            wall_size = (wall_thickness, room_size, wall_height)

        wall = create_primitive(
            prim_path=wall_path,
            primitive_type="Cuboid",
            primitive_props={
                "size": wall_size,
                "position": pos
            }
        )
        walls.append(wall_path)

    # Add furniture
    furniture = []
    table_path = "/World/table"
    table = create_primitive(
        prim_path=table_path,
        primitive_type="Cuboid",
        primitive_props={
            "size": (1.5, 0.8, 0.7),
            "position": (0, 0, 0.35)
        }
    )
    furniture.append(table_path)

    chair_path = "/World/chair"
    chair = create_primitive(
        prim_path=chair_path,
        primitive_type="Cuboid",
        primitive_props={
            "size": (0.4, 0.4, 0.8),
            "position": (-1, 0.5, 0.4)
        }
    )
    furniture.append(chair_path)

    print(f"Created indoor environment with {len(walls)} walls and {len(furniture)} furniture items")

    return walls, furniture


def create_outdoor_environment(world):
    """
    Create an outdoor environment with natural features
    """
    print("Creating outdoor environment...")

    # Create terrain with hills and valleys
    features = []

    # Add hills
    for i in range(3):
        hill_path = f"/World/hill_{i}"
        hill = create_primitive(
            prim_path=hill_path,
            primitive_type="Sphere",
            primitive_props={
                "radius": np.random.uniform(1, 3),
                "position": (np.random.uniform(-10, 10), np.random.uniform(-10, 10), np.random.uniform(1, 2)),
                "scale": (1, 1, 0.5)  # Flatten to create hills
            }
        )
        features.append(hill_path)

    # Add trees (represented as cylinders with spheres on top)
    for i in range(5):
        tree_base_path = f"/World/tree_trunk_{i}"
        tree_trunk = create_primitive(
            prim_path=tree_base_path,
            primitive_type="Cylinder",
            primitive_props={
                "radius": 0.2,
                "height": 2.0,
                "position": (np.random.uniform(-12, 12), np.random.uniform(-12, 12), 1.0)
            }
        )
        features.append(tree_base_path)

        tree_top_path = f"/World/tree_top_{i}"
        tree_top = create_primitive(
            prim_path=tree_top_path,
            primitive_type="Sphere",
            primitive_props={
                "radius": 1.0,
                "position": (np.random.uniform(-12, 12), np.random.uniform(-12, 12), 3.0)
            }
        )
        features.append(tree_top_path)

    print(f"Created outdoor environment with {len(features)} natural features")

    return features


def run_terrain_demo():
    """
    Run a complete terrain generation demonstration
    """
    print("Starting Isaac Sim Terrain Generation Demo...")

    try:
        # Create world instance
        world = World(stage_units_in_meters=1.0)

        # Create different types of terrains
        flat_terrain = create_flat_terrain(world)
        obstacles = create_rough_terrain(world)
        walls, furniture = create_indoor_environment(world)
        outdoor_features = create_outdoor_environment(world)

        print("\nTerrain generation demo completed successfully!")
        print(f"Created: 1 flat terrain, {len(obstacles)} obstacles, {len(walls)} walls, {len(furniture)} furniture, {len(outdoor_features)} outdoor features")

    except Exception as e:
        print(f"Error during terrain generation: {e}")
        return False

    return True


def main():
    """
    Main function to run the Isaac Sim terrain generation
    """
    print("Starting Isaac Sim Terrain Generation...")

    success = run_terrain_demo()

    if success:
        print("\nIsaac Sim Terrain Generation completed successfully!")
    else:
        print("\nFailed to execute terrain generation demo.")


if __name__ == "__main__":
    main()