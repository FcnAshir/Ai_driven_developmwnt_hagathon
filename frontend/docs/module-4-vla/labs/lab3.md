# Lab 3: Object Detection using YOLO/Isaac ROS (Conceptual)

## Objective

This lab provides a conceptual overview and simulated implementation of object detection, focusing on how a VLA system would integrate models like YOLO or NVIDIA Isaac ROS for real-time object recognition. Due to the complexity of setting up and running real-time deep learning models and GPU-accelerated frameworks, this lab will focus on the interaction patterns and data flow rather than a full, live implementation.

## Prerequisites

- Python 3.8+
- Basic understanding of computer vision and deep learning concepts.
- Conceptual understanding of ROS 2 topics and messages.

## Conceptual Overview

In a real-world VLA system for humanoid robotics, object detection is critical for grounding natural language commands in the physical environment. When an LLM generates a plan that requires interacting with an object (e.g., "pick up the red cube"), the robot needs to visually identify and locate that object.

**Key Technologies:**
-   **YOLO (You Only Look Once)**: A popular real-time object detection system known for its speed and accuracy.
-   **NVIDIA Isaac ROS**: A collection of hardware-accelerated ROS packages for robotics, including modules for perception (e.g., `isaac_ros_detectnet`, `isaac_ros_yolo`). Isaac ROS leverages NVIDIA GPUs (like those in a Jetson Orin) for high-performance inference.

**Data Flow:**
1.  **Camera Input**: The robot's camera captures image streams (e.g., as a `sensor_msgs/Image` ROS 2 topic).
2.  **Preprocessing**: Images might be preprocessed (e.g., resizing, normalization) before being fed to the detection model.
3.  **Object Detection Model**: The model processes the image and outputs detections, typically as bounding boxes, class labels, and confidence scores.
4.  **ROS 2 Output**: Detections are published as custom ROS 2 messages (e.g., `DetectedObjects` containing a list of `ObjectDetection` messages) on a dedicated topic.
5.  **LLM Integration**: The LLM-based planner subscribes to this topic, using the detected object information to refine its action plans.

## Steps (Simulated Implementation)

Since a full implementation requires specialized hardware and complex setups, we'll create a Python script that simulates object detection results based on a predefined scenario.

### 1. Create the Python Script

Create a new file named `simulated_object_detector.py` in `src/vla_agents/lab3/` with the following content:

```python
import json
import sys
import time

def simulate_object_detection(scene_description):
    """
    Simulates object detection in a given scene.
    In a real system, this would involve processing camera feeds with a deep learning model.
    """
    print(f"Simulating object detection for scene: \"{scene_description}\"\n")

    detected_objects = []

    # Simulate detections based on keywords in the scene description
    if "room" in scene_description.lower():
        detected_objects.append({
            "object_id": "red_cube",
            "class_name": "cube",
            "confidence": 0.95,
            "bounding_box": {"x": 0.2, "y": 0.3, "width": 0.1, "height": 0.1},
            "pose": {"position": {"x": 0.5, "y": -0.2, "z": 0.1}, "orientation": {"qx": 0, "qy": 0, "qz": 0, "qw": 1}}
        })
        detected_objects.append({
            "object_id": "blue_ball",
            "class_name": "ball",
            "confidence": 0.90,
            "bounding_box": {"x": 0.6, "y": 0.7, "width": 0.08, "height": 0.08},
            "pose": {"position": {"x": -0.3, "y": 0.4, "z": 0.05}, "orientation": {"qx": 0, "qy": 0, "qz": 0, "qw": 1}}
        })
    if "table" in scene_description.lower():
        detected_objects.append({
            "object_id": "coffee_cup",
            "class_name": "cup",
            "confidence": 0.88,
            "bounding_box": {"x": 0.1, "y": 0.1, "width": 0.05, "height": 0.07},
            "pose": {"position": {"x": 0.2, "y": 0.1, "z": 0.7}, "orientation": {"qx": 0, "qy": 0, "qz": 0, "qw": 1}}
        })

    time.sleep(1) # Simulate processing time

    return detected_objects

def display_detections(detections):
    print("--- Simulated Object Detections ---")
    if detections:
        print(json.dumps(detections, indent=2))
    else:
        print("No objects detected.")
    print("-----------------------------------")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python simulated_object_detector.py \"<scene description>\"")
        print("Example: python simulated_object_detector.py \"a room with a red cube and blue ball\"")
        sys.exit(1)

    scene = sys.argv[1]
    detections = simulate_object_detection(scene)
    display_detections(detections)
```

### 2. Run the Script

Execute the Python script from your terminal, providing a description of the scene:

```bash
python src/vla_agents/lab3/simulated_object_detector.py "a room with a red cube and blue ball"
```

Or for a scene with a table:

```bash
python src/vla_agents/lab3/simulated_object_detector.py "a table with a coffee cup"
```

### Expected Output (Example for "a room with a red cube and blue ball")

```
Simulating object detection for scene: "a room with a red cube and blue ball"

--- Simulated Object Detections ---
[
  {
    "object_id": "red_cube",
    "class_name": "cube",
    "confidence": 0.95,
    "bounding_box": {"x": 0.2, "y": 0.3, "width": 0.1, "height": 0.1},
    "pose": {"position": {"x": 0.5, "y": -0.2, "z": 0.1}, "orientation": {"qx": 0, "qy": 0, "qz": 0, "qw": 1}}
  },
  {
    "object_id": "blue_ball",
    "class_name": "ball",
    "confidence": 0.90,
    "bounding_box": {"x": 0.6, "y": 0.7, "width": 0.08, "height": 0.08},
    "pose": {"position": {"x": -0.3, "y": 0.4, "z": 0.05}, "orientation": {"qx": 0, "qy": 0, "qz": 0, "qw": 1}}
  }
]
-----------------------------------
```

## Troubleshooting

-   **No detections**: Ensure your scene description contains keywords matching the simulated detection logic.
-   **JSON formatting issues**: Double-check the Python script for correct JSON output structure.

## Next Steps

This lab provided a conceptual and simulated understanding of object detection. In the next labs, we will integrate this detection capability with LLM planning and ROS 2 action execution to create a more complete VLA pipeline.
