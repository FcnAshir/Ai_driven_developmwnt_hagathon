# Lab 2: LLM Action Planning

## Overview
This lab demonstrates how to use Large Language Models (LLMs) to generate action plans from natural language commands. You'll create a simulated LLM that converts commands into ROS 2 action sequences.

## Learning Objectives
- Understand LLM role in VLA systems
- Generate action plans from natural language
- Structure action plans for robotic execution

## Files
- `llm_action_planner.py`: Implementation of LLM-based action planning

## Theory
Large Language Models serve as the reasoning component in VLA systems, converting high-level natural language commands into executable action sequences. The LLM interprets user intent and creates a plan that can be executed by the robot.

## Implementation
The implementation includes:
- Natural language command parsing
- Action plan generation with ROS 2 action structure
- Support for complex multi-step commands
- JSON-formatted plan output

## Running the Demo
```bash
python llm_action_planner.py "<natural language command>"
# Example:
python llm_action_planner.py "robot, move forward five meters"
```

## Exercises
1. Add support for more complex action types (navigation, manipulation, etc.)
2. Implement spatial reasoning in command interpretation
3. Add error handling for unknown commands