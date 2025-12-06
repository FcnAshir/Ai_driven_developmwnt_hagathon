import json
import sys
import os

def generate_action_plan(natural_language_command):
    """
    Simulates an LLM generating a ROS 2 action plan from a natural language command.
    In a real system, this would involve API calls to an LLM.
    """
    print(f"LLM processing command: \"{natural_language_command}\"\n")

    # Simulate LLM's understanding and plan generation
    command_lower = natural_language_command.lower()
    action_plan = []

    if "move forward" in command_lower:
        distance = "5 meters" if "five meters" in command_lower else "default distance"
        action_plan.append({"action": "move_linear", "target": {"x": float(distance.split()[0]), "y": 0.0, "z": 0.0}})
    if "turn left" in command_lower:
        angle = "90 degrees" if "ninety degrees" in command_lower else "default angle"
        action_plan.append({"action": "rotate_relative", "angle": float(angle.split()[0])})
    if "pick up" in command_lower and "red cube" in command_lower:
        action_plan.append({"action": "grasp_object", "object_id": "red_cube", "location": "current"})
    if "clean the room" in command_lower:
        action_plan.extend([
            {"action": "navigate_to", "target": "kitchen"},
            {"action": "grasp_object", "object_id": "dirty_plate", "location": "table"},
            {"action": "place_object", "object_id": "dirty_plate", "location": "sink"},
            {"action": "navigate_to", "target": "living_room"},
            {"action": "grasp_object", "object_id": "toy_car", "location": "floor"},
            {"action": "place_object", "object_id": "toy_car", "location": "toy_box"}
        ])

    if not action_plan:
        action_plan.append({"action": "unknown_command", "command": natural_language_command})

    return action_plan

def display_plan(plan):
    print("--- Generated ROS 2 Action Plan ---")
    print(json.dumps(plan, indent=2))
    print("----------------------------------")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python llm_action_planner.py \"<natural language command>\"")
        print("Example: python llm_action_planner.py \"robot, move forward five meters\"")
        sys.exit(1)

    command = sys.argv[1]
    plan = generate_action_plan(command)
    display_plan(plan)
