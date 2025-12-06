# Lab 5: Complete VLA Room Cleaning Task

## Overview
This lab demonstrates a complete, multi-step VLA task: room cleaning. You'll implement a system that processes complex commands, manages multiple objects, and executes a sequence of actions to clean a room.

## Learning Objectives
- Implement complex multi-step VLA tasks
- Manage object state changes during execution
- Handle iterative planning and execution

## Files
- `clean_room_vla.py`: Complete VLA system for room cleaning task

## Theory
Complex VLA tasks require iterative planning where the system continuously updates its understanding of the world state and re-plans actions accordingly. This lab demonstrates how to manage state changes and adapt plans during execution.

## Implementation
The implementation includes:
- Complex command processing ("clean the room")
- Multi-object detection and management
- Iterative action execution with state updates
- Object removal tracking after placement
- Completion verification

## Running the Demo
```bash
python clean_room_vla.py <path_to_audio_file> "<initial scene description>"
# Example:
python clean_room_vla.py mock_command.wav "a room with a dirty plate and a toy car"
```

## Exercises
1. Add support for more object types and locations
2. Implement path planning to avoid obstacles during navigation
3. Add user feedback during task execution