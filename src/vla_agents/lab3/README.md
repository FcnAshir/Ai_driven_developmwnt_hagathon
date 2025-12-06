# Lab 3: Simulated Object Detection

## Overview
This lab demonstrates object detection in VLA systems. You'll create a simulated object detector that identifies objects in a scene and provides their locations and properties.

## Learning Objectives
- Understand object detection in robotics
- Create simulated detection outputs
- Format detection results for VLA systems

## Files
- `simulated_object_detector.py`: Implementation of simulated object detection

## Theory
Object detection is the vision component of VLA systems, identifying objects in the environment and their spatial relationships. This information is crucial for the LLM to generate appropriate action plans.

## Implementation
The implementation includes:
- Scene description processing
- Simulated object detection with confidence scores
- 3D pose estimation for detected objects
- Bounding box information for 2D localization

## Running the Demo
```bash
python simulated_object_detector.py "<scene description>"
# Example:
python simulated_object_detector.py "a room with a red cube and blue ball"
```

## Exercises
1. Add more object classes to the detector
2. Implement realistic confidence scoring
3. Add spatial relationship detection between objects