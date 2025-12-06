# Lab 5: Build Final VLA Project: “Clean the Room” Pipeline (Simulated)

## Objective

This lab integrates all the conceptual components of the VLA pipeline developed in previous labs to simulate a complex robot task: "Clean the room." This will involve a sequence of voice command, LLM planning, object detection, and simulated robot actions.

## Prerequisites

- Python 3.8+
- Conceptual understanding of ROS 2 actions and state.
- Completion of Lab 1 (Whisper), Lab 2 (LLM Planner), Lab 3 (Object Detection), and Lab 4 (Robot Pick Object).

## Conceptual Overview

The "Clean the room" pipeline is a multi-step task that demonstrates the full capabilities of a VLA system. The robot needs to:

1.  **Receive High-Level Command**: User says, "Robot, clean the room."
2.  **Transcribe Command**: Whisper converts speech to text.
3.  **LLM Global Planning**: The LLM interprets "clean the room" as a sequence of sub-tasks:
    -   Identify all "dirty" objects.
    -   Determine target locations for each object (e.g., "dirty plate" to "sink", "toy car" to "toy box").
    -   Generate a sequence of navigation and manipulation actions to achieve this.
4.  **Iterative Perception & Action**: For each object:
    -   **Object Detection**: Identify the object and its pose.
    -   **LLM Local Planning**: Generate specific actions (navigate, approach, grasp, place) for that object.
    -   **ROS 2 Execution**: Execute the actions.
5.  **Task Completion**: Report when the room is clean.

## Steps (Simulated Full VLA Pipeline Script)

We will create a Python script that orchestrates the simulated components from all previous labs to demonstrate this integrated, iterative flow.

### 1. Create the Python Script

Create a new file named `clean_room_vla.py` in `src/vla_agents/lab5/` with the following content:

```python
import sys
import os
import time
import json

# Mock functions from previous labs
def mock_whisper_transcribe(audio_file):
    print(f"[Whisper] Transcribing audio from {audio_file}...")
    time.sleep(0.5)
    return "robot, clean the room"

def mock_object_detector_get_scene_info(scene_description):
    print(f"[Detector] Simulating detection for scene: \"{scene_description}\"")
    time.sleep(1)
    if "room" in scene_description.lower():
        return {
            "dirty_plate": {"location": [0.8, 0.5, 0.7], "class": "plate"},
            "toy_car": {"location": [-0.5, -0.3, 0.1], "class": "toy"}
        }
    return {}

def mock_llm_generate_global_plan(natural_language_command, detected_objects_info):
    print(f"[LLM] Generating global plan for: \"{natural_language_command}\" with objects: {detected_objects_info}")
    time.sleep(1.5)

    global_plan = []
    if "clean the room" in natural_language_command.lower() and detected_objects_info:
        for obj_id, obj_info in detected_objects_info.items():
            target_location = "sink" if obj_info["class"] == "plate" else "toy_box"
            global_plan.extend([
                {"action": "navigate_to", "target_object": obj_id, "location": obj_info["location"]},
                {"action": "approach_object", "target_object": obj_id},
                {"action": "grasp_object", "target_object": obj_id},
                {"action": "navigate_to", "target_location": target_location},
                {"action": "place_object", "target_object": obj_id, "location": target_location}
            ])
    else:
        global_plan.append({"action": "unknown_command", "command": natural_language_command})
    return global_plan

def mock_ros2_execute_action(action):
    print(f"[ROS2] Executing action: {action}")
    time.sleep(1.0) # Simulate robot movement/manipulation time
    if action["action"] in ["grasp_object", "place_object"]:
        print(f"[ROS2] Successfully completed {action["action"]} for {action.get("target_object", "unknown")}")
    return True

def main(audio_file_path, initial_scene_description):
    print("--- Starting VLA "Clean the Room" Demo ---\n")

    # 1. Simulate Voice Command (Whisper)
    transcribed_text = mock_whisper_transcribe(audio_file_path)
    print(f"[VLA Pipeline] Transcribed: \"{transcribed_text}\"\n")

    # 2. Simulate Initial Object Detection
    current_detected_objects = mock_object_detector_get_scene_info(initial_scene_description)
    print(f"[VLA Pipeline] Initial Detected Objects: {current_detected_objects}\n")

    # 3. Simulate LLM Global Planning
    global_action_plan = mock_llm_generate_global_plan(transcribed_text, current_detected_objects)
    print(f"[VLA Pipeline] Generated Global Plan: {global_action_plan}\n")

    # 4. Simulate ROS 2 Action Execution (Iterative)
    print("Starting iterative action execution...")
    for i, action in enumerate(global_action_plan):
        print(f"Executing step {i+1}/{len(global_action_plan)}: {action}")
        success = mock_ros2_execute_action(action)
        if not success:
            print(f"Action {action["action"]} failed. Halting.")
            break

        # In a real system, detection would update after each action
        # For simulation, we assume objects are removed/moved successfully
        if action["action"] == "place_object":
            obj_id_to_remove = action["target_object"]
            if obj_id_to_remove in current_detected_objects:
                del current_detected_objects[obj_id_to_remove]
                print(f"[Simulation] Removed {obj_id_to_remove} from scene.")

    print("\n--- VLA "Clean the Room" Demo Finished ---")
    if not current_detected_objects:
        print("Room is clean! All objects have been tidied.")
    else:
        print(f"Remaining objects: {current_detected_objects}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python clean_room_vla.py <path_to_mock_audio_file> \"<initial scene description>\"")
        print("Example: python clean_room_vla.py mock_command.wav \"a room with a dirty plate and a toy car\"")
        sys.exit(1)

    mock_audio = sys.argv[1]
    initial_scene = sys.argv[2]
    main(mock_audio, initial_scene)
```

### 2. Create a Mock Audio File

Similar to Lab 4, create an empty file named `mock_command.wav` in the `src/vla_agents/lab5/` directory.

```bash
touch src/vla_agents/lab5/mock_command.wav
```

### 3. Run the Script

Execute the Python script from your terminal, providing the mock audio file and an initial scene description:

```bash
python src/vla_agents/lab5/clean_room_vla.py src/vla_agents/lab5/mock_command.wav "a room with a dirty plate and a toy car"
```

### Expected Output (Example)

```
--- Starting VLA "Clean the Room" Demo ---

[Whisper] Transcribing audio from src/vla_agents/lab5/mock_command.wav...
[VLA Pipeline] Transcribed: "robot, clean the room"

[Detector] Simulating detection for scene: "a room with a dirty plate and a toy car"
[VLA Pipeline] Initial Detected Objects: {'dirty_plate': {'location': [0.8, 0.5, 0.7], 'class': 'plate'}, 'toy_car': {'location': [-0.5, -0.3, 0.1], 'class': 'toy'}}

[LLM] Generating global plan for: "robot, clean the room" with objects: {'dirty_plate': {'location': [0.8, 0.5, 0.7], 'class': 'plate'}, 'toy_car': {'location': [-0.5, -0.3, 0.1], 'class': 'toy'}}
[VLA Pipeline] Generated Global Plan: [{'action': 'navigate_to', 'target_object': 'dirty_plate', 'location': [0.8, 0.5, 0.7]}, {'action': 'approach_object', 'target_object': 'dirty_plate'}, {'action': 'grasp_object', 'target_object': 'dirty_plate'}, {'action': 'navigate_to', 'target_location': 'sink'}, {'action': 'place_object', 'target_object': 'dirty_plate', 'location': 'sink'}, {'action': 'navigate_to', 'target_object': 'toy_car', 'location': [-0.5, -0.3, 0.1]}, {'action': 'approach_object', 'target_object': 'toy_car'}, {'action': 'grasp_object', 'target_object': 'toy_car'}, {'action': 'navigate_to', 'target_location': 'toy_box'}, {'action': 'place_object', 'target_object': 'toy_car', 'location': 'toy_box'}]

Starting iterative action execution...
Executing step 1/10: {'action': 'navigate_to', 'target_object': 'dirty_plate', 'location': [0.8, 0.5, 0.7]}
[ROS2] Executing action: {'action': 'navigate_to', 'target_object': 'dirty_plate', 'location': [0.8, 0.5, 0.7]}
Executing step 2/10: {'action': 'approach_object', 'target_object': 'dirty_plate'}
[ROS2] Executing action: {'action': 'approach_object', 'target_object': 'dirty_plate'}
Executing step 3/10: {'action': 'grasp_object', 'target_object': 'dirty_plate'}
[ROS2] Executing action: {'action': 'grasp_object', 'target_object': 'dirty_plate'}
[ROS2] Successfully completed grasp_object for dirty_plate
Executing step 4/10: {'action': 'navigate_to', 'target_location': 'sink'}
[ROS2] Executing action: {'action': 'navigate_to', 'target_location': 'sink'}
Executing step 5/10: {'action': 'place_object', 'target_object': 'dirty_plate', 'location': 'sink'}
[ROS2] Executing action: {'action': 'place_object', 'target_object': 'dirty_plate', 'location': 'sink'}
[ROS2] Successfully completed place_object for dirty_plate
[Simulation] Removed dirty_plate from scene.
Executing step 6/10: {'action': 'navigate_to', 'target_object': 'toy_car', 'location': [-0.5, -0.3, 0.1]}
[ROS2] Executing action: {'action': 'navigate_to', 'target_object': 'toy_car', 'location': [-0.5, -0.3, 0.1]}
Executing step 7/10: {'action': 'approach_object', 'target_object': 'toy_car'}
[ROS2] Executing action: {'action': 'approach_object', 'target_object': 'toy_car'}
Executing step 8/10: {'action': 'grasp_object', 'target_object': 'toy_car'}
[ROS2] Executing action: {'action': 'grasp_object', 'target_object': 'toy_car'}
[ROS2] Successfully completed grasp_object for toy_car
Executing step 9/10: {'action': 'navigate_to', 'target_location': 'toy_box'}
[ROS2] Executing action: {'action': 'navigate_to', 'target_location': 'toy_box'}
Executing step 10/10: {'action': 'place_object', 'target_object': 'toy_car', 'location': 'toy_box'}
[ROS2] Executing action: {'action': 'place_object', 'target_object': 'toy_car', 'location': 'toy_box'}
[ROS2] Successfully completed place_object for toy_car
[Simulation] Removed toy_car from scene.

--- VLA "Clean the Room" Demo Finished ---
Room is clean! All objects have been tidied.
```

## Troubleshooting

-   **Unexpected plan**: Review `mock_llm_generate_global_plan` to ensure it correctly decomposes complex commands.
-   **Integration issues**: Verify that data flows correctly between the mock components.
-   **Simulation logic**: Adjust the simulation logic in `main` to handle more complex scenarios or object interactions.

## Next Steps

This marks the completion of the core VLA module implementation. The remaining tasks for this module involve drafting troubleshooting, summary, and further reading sections.
