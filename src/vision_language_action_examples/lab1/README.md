# Lab 1: Basic Vision-Language-Action (VLA) System

## Overview
This lab introduces the fundamental concepts of Vision-Language-Action (VLA) systems for robotics. You'll implement a simple VLA pipeline that processes visual input, understands natural language commands, and generates robot actions.

## Learning Objectives
- Understand the VLA architecture components
- Implement basic vision processing
- Create simple language understanding
- Plan robot actions based on perception and commands

## Files
- `simple_vla_demo.py`: Basic VLA implementation

## Theory
VLA systems combine three key components:
1. **Vision**: Object detection and scene understanding
2. **Language**: Natural language command interpretation
3. **Action**: Robot command generation and execution

## Implementation
The basic VLA system includes:
- Vision processor with object detection
- Language processor with command parsing
- Action planner with command generation

## Running the Demo
```bash
python3 simple_vla_demo.py
```

## Exercises
1. Modify the vision processor to detect different objects
2. Add new command patterns to the language processor
3. Implement additional action types in the action planner