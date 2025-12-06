# Lab 2: LLM-based Text to ROS Action Plan Conversion

## Objective

This lab will guide you through using a Large Language Model (LLM) to convert natural language text commands into a structured ROS 2 action plan. We will simulate an LLM's response to generate a sequence of robot actions.

## Prerequisites

- Python 3.8+
- `pip` package manager
- Basic understanding of ROS 2 concepts (nodes, actions)

## Steps

### 1. Create the Python Script

Create a new file named `llm_action_planner.py` in `src/vla_agents/lab2/` with the following content:

```python
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
```

### 2. Run the Script

Execute the Python script from your terminal, providing a natural language command in quotes:

```bash
python src/vla_agents/lab2/llm_action_planner.py "robot, move forward five meters"
```

Or for a more complex command:

```bash
python src/vla_agents/lab2/llm_action_planner.py "robot, pick up the red cube and turn left ninety degrees"
```

### Expected Output (Example for "robot, move forward five meters")

```
LLM processing command: "robot, move forward five meters"

--- Generated ROS 2 Action Plan ---
[
  {
    "action": "move_linear",
    "target": {
      "x": 5.0,
      "y": 0.0,
      "z": 0.0
    }
  }
]
----------------------------------
```

## Troubleshooting

-   **`json.decoder.JSONDecodeError`**: Ensure your Python script is correctly generating valid JSON output.
-   **Incorrect plan**: Adjust the `generate_action_plan` logic in the script to better interpret commands and generate appropriate actions.
-   **Missing actions**: Expand the `if` conditions in the script to cover more natural language commands and map them to robot actions.

## Next Steps

This lab demonstrated converting natural language to an action plan. In the next labs, we will integrate this with actual object detection and ROS 2 execution to make a robot perform physical tasks.
