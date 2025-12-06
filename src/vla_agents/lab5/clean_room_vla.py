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
        print(f"[ROS2] Successfully completed {action["action"]}. ")
    return True

def main(audio_file_path, initial_scene_description):
    print("--- Starting VLA \"Clean the Room\" Demo ---\n")

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

    print("\n--- VLA \"Clean the Room\" Demo Finished ---")
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
