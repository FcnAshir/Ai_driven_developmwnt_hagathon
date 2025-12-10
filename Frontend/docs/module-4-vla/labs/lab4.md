# Lab 4: Robot Picks an Object After Verbal Instruction (Simulated)

## Objective

This lab conceptually demonstrates how a humanoid robot can pick up an object based on a verbal instruction, integrating the components from previous labs: voice command transcription (Whisper), LLM-based action planning, and simulated object detection. We will focus on the control flow and integration logic rather than a full real-world robotics implementation.

## Prerequisites

- Python 3.8+
- Conceptual understanding of ROS 2 actions and state.
- Completion of Lab 1 (Whisper) and Lab 2 (LLM Planner) and Lab 3 (Object Detection).

## Conceptual Overview

In a full VLA pipeline, the process for a robot to pick up an object after a verbal instruction would involve:

1.  **Verbal Command**: User speaks a command like "Robot, pick up the red cube."
2.  **Speech-to-Text**: Whisper transcribes the command to "pick up the red cube."
3.  **LLM Planning**: The LLM processes the text, queries the object detection system for the "red cube"'s location, and generates a plan like:
    -   `navigate_to(red_cube_location)`
    -   `approach_object(red_cube)`
    -   `grasp(red_cube)`
    -   `lift_object(red_cube)`
4.  **Object Detection**: The robot's vision system (e.g., YOLO/Isaac ROS) continuously detects objects, providing their locations and poses.
5.  **ROS 2 Action Execution**: The plan's actions are translated into ROS 2 goals (e.g., `MoveToGoal`, `GraspGoal`) and sent to the robot's locomotion and manipulation controllers.
6.  **Sensing and Feedback**: Throughout execution, the robot uses its sensors (joint encoders, force sensors, cameras) to monitor progress and provide feedback, allowing the LLM-based system to adjust the plan if necessary.

## Steps (Simulated Integration Script)

We will create a Python script that orchestrates the simulated components from previous labs to demonstrate this integrated flow.

### 1. Create the Python Script

Create a new file named `robot_pick_object.py` in `src/vla_agents/lab4/` with the following content:

```python
import sys
import os
import time

# Assuming these are available from previous labs or a common utility module
# For this simulation, we'll use placeholder functions.

def mock_whisper_transcribe(audio_file):
    # Simulate Whisper transcription
    print(f"[Whisper] Transcribing audio from {audio_file}...")
    time.sleep(0.5)
    # Example hardcoded response for demonstration
    return "robot, pick up the red cube"

def mock_llm_generate_plan(natural_language_command, detected_objects_info):
    # Simulate LLM action plan generation
    print(f"[LLM] Generating plan for: \"{natural_language_command}\" with objects: {detected_objects_info}")
    time.sleep(1)

    plan = []
    if "pick up the red cube" in natural_language_command.lower() and "red_cube" in detected_objects_info:
        plan.append({"action": "navigate_to", "target_object": "red_cube"})
        plan.append({"action": "approach_object", "target_object": "red_cube"})
        plan.append({"action": "grasp_object", "target_object": "red_cube"})
        plan.append({"action": "lift_object", "target_object": "red_cube"})
    elif "move forward" in natural_language_command.lower():
        plan.append({"action": "move_linear", "distance": 5.0})
    else:
        plan.append({"action": "unknown_command", "command": natural_language_command})
    return plan

def mock_object_detector_get_scene_info(scene_description):
    # Simulate object detection
    print(f"[Detector] Simulating detection for scene: \"{scene_description}\"")
    time.sleep(0.7)
    # Example hardcoded response
    if "red cube" in scene_description.lower():
        return {"red_cube": {"location": [0.5, -0.2, 0.1], "class": "cube"}}
    return {}

def mock_ros2_execute_action(action):
    # Simulate ROS 2 action execution
    print(f"[ROS2] Executing action: {action}")
    time.sleep(1.5) # Simulate robot movement time
    if action["action"] == "grasp_object":
        print(f"[ROS2] Successfully grasped {action["target_object"]}")
        return True
    return True

def main(audio_file_path, scene_description):
    print("--- Starting VLA Robot Pick Object Demo ---\n")

    # 1. Simulate Voice Command (Whisper)
    transcribed_text = mock_whisper_transcribe(audio_file_path)
    print(f"[VLA Pipeline] Transcribed: \"{transcribed_text}\"\n")

    # 2. Simulate Object Detection
    detected_objects_info = mock_object_detector_get_scene_info(scene_description)
    print(f"[VLA Pipeline] Detected Objects: {detected_objects_info}\n")

    # 3. Simulate LLM Planning
    action_plan = mock_llm_generate_plan(transcribed_text, detected_objects_info)
    print(f"[VLA Pipeline] Generated Plan: {action_plan}\n")

    # 4. Simulate ROS 2 Action Execution
    print("Starting action execution...")
    for i, action in enumerate(action_plan):
        print(f"Executing step {i+1}/{len(action_plan)}: {action}")
        success = mock_ros2_execute_action(action)
        if not success:
            print(f"Action {action["action"]} failed. Halting.")
            break
    print("\n--- VLA Robot Pick Object Demo Finished ---")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python robot_pick_object.py <path_to_mock_audio_file> \"<scene description>\"")
        print("Example: python robot_pick_object.py mock_command.wav \"a room with a red cube on a table\"")
        sys.exit(1)

    mock_audio = sys.argv[1]
    scene = sys.argv[2]
    main(mock_audio, scene)
```

### 2. Create a Mock Audio File

For this simulated lab, you don't need a real audio file. Create an empty file named `mock_command.wav` in the `src/vla_agents/lab4/` directory. Its presence will satisfy the script's argument requirement.

```bash
touch src/vla_agents/lab4/mock_command.wav
```

### 3. Run the Script

Execute the Python script from your terminal, providing the mock audio file and a scene description:

```bash
python src/vla_agents/lab4/robot_pick_object.py src/vla_agents/lab4/mock_command.wav "a room with a red cube on a table"
```

### Expected Output (Example)

```
--- Starting VLA Robot Pick Object Demo ---

[Whisper] Transcribing audio from src/vla_agents/lab4/mock_command.wav...
[VLA Pipeline] Transcribed: "robot, pick up the red cube"

[Detector] Simulating detection for scene: "a room with a red cube on a table"
[VLA Pipeline] Detected Objects: {'red_cube': {'location': [0.5, -0.2, 0.1], 'class': 'cube'}}

[LLM] Generating plan for: "robot, pick up the red cube" with objects: {'red_cube': {'location': [0.5, -0.2, 0.1], 'class': 'cube'}}
[VLA Pipeline] Generated Plan: [{'action': 'navigate_to', 'target_object': 'red_cube'}, {'action': 'approach_object', 'target_object': 'red_cube'}, {'action': 'grasp_object', 'target_object': 'red_cube'}, {'action': 'lift_object', 'target_object': 'red_cube'}]

Starting action execution...
Executing step 1/4: {'action': 'navigate_to', 'target_object': 'red_cube'}
[ROS2] Executing action: {'action': 'navigate_to', 'target_object': 'red_cube'}
Executing step 2/4: {'action': 'approach_object', 'target_object': 'red_cube'}
[ROS2] Executing action: {'action': 'approach_object', 'target_object': 'red_cube'}
Executing step 3/4: {'action': 'grasp_object', 'target_object': 'red_cube'}
[ROS2] Executing action: {'action': 'grasp_object', 'target_object': 'red_cube'}
[ROS2] Successfully grasped red_cube
Executing step 4/4: {'action': 'lift_object', 'target_object': 'red_cube'}
[ROS2] Executing action: {'action': 'lift_object', 'target_object': 'red_cube'}

--- VLA Robot Pick Object Demo Finished ---
```

## Troubleshooting

-   **Script not running**: Ensure correct Python path and arguments.
-   **Unexpected plan**: Modify `mock_llm_generate_plan` to improve its logic.
-   **Action failure**: Extend `mock_ros2_execute_action` to simulate different success/failure conditions.

## Next Steps

This lab provided an integrated simulation of the VLA pipeline for a robot to pick an object. The final lab will build a more comprehensive VLA project combining all these elements into a complex task like "Clean the room."
