# Book Architecture Diagrams

## Overall Book Architecture

### Humanoid Robot Stack
```
Sensors → ROS 2 → Isaac → VLA → Control Loops
```

### Digital Twin Stack
```
Gazebo/Unity → ROS 2 Bridge → Isaac → Training
```

### VLA Stack
```
Whisper → LLM Planner → Vision → ROS 2 Actions
```

## Module Integration Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Module 1      │    │   Module 2       │    │   Module 3      │
│   ROS 2         │    │   Digital Twin   │    │   Isaac         │
│   (Nervous)     │───▶│   (Simulation)   │───▶│   (AI Brain)    │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Module 4      │    │   Capstone       │    │   Deployment    │
│   VLA           │    │   Integration    │    │   Pipeline      │
│   (VLA)         │    │   (Full System)  │    │   (Real/Hybrid) │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## Detailed System Architecture

### Full VLA Pipeline Architecture
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Voice Input   │───▶│  Whisper ASR    │───▶│  LLM Planner    │
│   (Microphone)  │    │  (Speech → Txt) │    │  (Plan Gen)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                                       │
┌─────────────────┐    ┌─────────────────┐             │
│   Camera/       │───▶│  Vision        │─────────────┤
│   Sensor Data   │    │  Processing    │             │
└─────────────────┘    └─────────────────┘             │
         │                       │                      │
         ▼                       ▼                      ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Object        │    │  Scene          │    │  ROS 2 Action   │
│   Detection     │    │  Understanding  │    │  Execution      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                      │
         └───────────────────────┼──────────────────────┘
                                 ▼
                    ┌─────────────────────────┐
                    │   Humanoid Robot        │
                    │   (Physical/Simulated)  │
                    └─────────────────────────┘
```

### Simulation-to-Reality Pipeline
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Gazebo        │    │   Isaac Sim     │    │   Unity         │
│   (Physics)     │    │   (Rendering)   │    │   (Visual)      │
└─────────┬───────┘    └───────┬─────────┘    └────────┬────────┘
          │                    │                       │
          └────────────────────┼───────────────────────┘
                               ▼
                    ┌─────────────────┐    ┌─────────────────┐
                    │   ROS 2 Bridge  │───▶│   Isaac ROS     │
                    │   (Integration) │    │   (Perception)  │
                    └─────────────────┘    └─────────────────┘
                               │                       │
                               ▼                       ▼
                    ┌─────────────────────────────────────────┐
                    │           Real Robot/Hardware           │
                    └─────────────────────────────────────────┘
```