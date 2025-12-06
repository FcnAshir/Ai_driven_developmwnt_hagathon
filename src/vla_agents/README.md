# VLA Agents Package

This package provides Vision-Language-Action (VLA) agents for humanoid robotics applications, demonstrating how to integrate voice commands, object detection, and robotic actions.

## Overview

The VLA Agents package includes 5 labs that progressively build a complete VLA system:

- **Lab 1**: Voice command processing with Whisper ASR
- **Lab 2**: LLM-based action planning
- **Lab 3**: Simulated object detection
- **Lab 4**: Complete object picking pipeline
- **Lab 5**: Complex room cleaning task

## Components

### Voice Command Processing
- Speech-to-text conversion using Whisper
- Command transcription and validation

### Language Understanding
- LLM-based command interpretation
- Action plan generation
- Multi-step planning capabilities

### Vision Processing
- Object detection and localization
- Scene understanding
- Spatial relationship analysis

### Action Execution
- ROS 2 action planning
- Robot control interfaces
- Task execution monitoring

## Usage

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select vla_agents
source install/setup.bash
```

### 2. Run Individual Labs
```bash
# Lab 1: Voice command processing
python3 src/vla_agents/vla_agents/whisper_command.py <audio_file>

# Lab 2: LLM action planning
python3 src/vla_agents/vla_agents/llm_action_planner.py "robot, move forward five meters"

# Lab 3: Object detection
python3 src/vla_agents/vla_agents/simulated_object_detector.py "a room with a red cube"

# Lab 4: Object picking
python3 src/vla_agents/vla_agents/robot_pick_object.py mock_command.wav "a room with a red cube"

# Lab 5: Room cleaning
python3 src/vla_agents/vla_agents/clean_room_vla.py mock_command.wav "a room with a dirty plate and toy car"
```

### 3. Or Use ROS 2 Launch
```bash
ros2 run vla_agents whisper_command <audio_file>
ros2 run vla_agents llm_action_planner "robot, move forward"
```

## Files Structure

```
vla_agents/
├── vla_agents/
│   ├── __init__.py
│   ├── whisper_command.py
│   ├── llm_action_planner.py
│   ├── simulated_object_detector.py
│   ├── robot_pick_object.py
│   └── clean_room_vla.py
├── lab1/
├── lab2/
├── lab3/
├── lab4/
├── lab5/
├── launch/
├── test/
├── package.xml
├── setup.py
├── CMakeLists.txt
└── README.md
```

## Dependencies

- OpenAI Whisper (for speech recognition)
- ROS 2 (for robot communication)
- NumPy, JSON, and other standard Python libraries

## Troubleshooting

- Ensure Whisper is properly installed: `pip install openai-whisper`
- Check that ROS 2 environment is sourced
- Verify audio files exist for voice command processing