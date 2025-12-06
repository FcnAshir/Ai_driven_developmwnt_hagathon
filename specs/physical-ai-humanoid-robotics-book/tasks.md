# Tasks: Physical AI & Humanoid Robotics — 4-Module Technical Book

**Input**: Design documents from `/specs/physical-ai-humanoid-robotics-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

-   **[P]**: Can run in parallel (different files, no dependencies)
-   **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
-   Include exact file paths in descriptions

## Path Conventions

-   **Single project**: `src/`, `tests/` at repository root
-   **Web app**: `backend/src/`, `frontend/src/`
-   **Mobile**: `api/src/`, `ios/src/` or `android/src/`
-   Paths shown below assume single project - adjust based on plan.md structure

---

## Phase 1: Res`earch

**Purpose**: Initial information gathering and architectural diagramming.

-   [X] T001 Gather ROS 2 primary sources for Module 1 documentation. docs/module-1-ros2/
-   [X] T002 Gather Gazebo/Unity primary sources for Module 2 documentation. docs/module-2-digital-twin/
-   [X] T003 Gather NVIDIA Isaac Sim/Isaac ROS primary sources for Module 3 documentation. docs/module-3-isaac-sim-ros/
-   [X] T004 Gather VLA (Whisper, LLM, Vision) primary sources for Module 4 documentation. docs/module-4-vla/
-   [X] T005 Build initial reference architecture diagrams for the overall book structure. static/diagrams/book_architecture.png

---

## Phase 2: Foundation (Book Skeleton & Global Sections)

**Purpose**: Establishing the Docusaurus book structure and writing global, cross-cutting sections.

-   [X] T006 Write Preface for the book. docs/
-   [X] T007 Write Physical AI Overview for the book. docs/physical-ai-overview.md
-   [X] T008 Create Docusaurus book skeleton and folder structure. docs/
-   [X] T009 Validate hardware requirements section for both physical and cloud setups. docs/hardware-appendix.md
-   [X] T010 Create placeholder for Weekly Breakdown (13-Week Roadmap). docs/weekly-breakdown.md
-   [X] T011 Create placeholder for Capstone Project Specification. docs/capstone-project.md
-   [X] T012 Create placeholder for Sim-to-Real Considerations. docs/sim-to-real.md
-   [X] T013 Create placeholder for Glossary + References. docs/glossary.md

**Checkpoint**: Foundation ready - module content development can now begin.

---

## Phase 3: Module 1 - The Robotic Nervous System (ROS 2)

**Goal**: Develop content for the ROS 2 chapter, including core concepts, architecture, and hands-on labs for building and connecting ROS 2 components.

**Independent Test**: Students can successfully build, run, and connect ROS 2 nodes, topics, services, actions, and deploy URDF-based humanoid models in a simulated environment, verifying all functional requirements are met and labs are reproducible.

### Implementation for Module 1

-   [X] T014 [P] [US1] Draft Module 1: ROS 2 Overview + Learning Outcomes. docs/module-1-ros2/index.md
-   [X] T015 [P] [US1] Draft Module 1: ROS 2 Core Concepts. docs/module-1-ros2/index.md
-   [X] T016 [P] [US1] Draft Module 1: ROS 2 Architecture + Diagrams. docs/module-1-ros2/index.md
-   [X] T017 [P] [US1] Draft Module 1: ROS 2 Deep Technical Foundation (including DDS and lifecycle management). docs/module-1-ros2/index.md
-   [X] T018 [P] [US1] Draft Module 1: ROS 2 Practical Tutorials. docs/module-1-ros2/index.md
-   [X] T019 [P] [US1] Implement Lab 1: Create a ROS 2 package, code in `src/ros2_package_examples/lab1/`. Docs: `docs/module-1-ros2/labs/lab1.md`
-   [X] T020 [P] [US1] Implement Lab 2: Publish and subscribe to topics, code in `src/ros2_package_examples/lab2/`. Docs: `docs/module-1-ros2/labs/lab2.md`
-   [X] T021 [P] [US2] Implement Lab 3: Build a humanoid URDF (torso + arms + legs), code in `src/ros2_package_examples/lab3/`. Docs: `docs/module-1-ros2/labs/lab3.md`
-   [X] T022 [P] [US3] Implement Lab 4: Control a joint using rclpy, code in `src/ros2_package_examples/lab4/`. Docs: `docs/module-1-ros2/labs/lab4.md`
-   [X] T023 [P] [US4] Implement Lab 5: Connect a local LLM agent to ROS actions, code in `src/ros2_package_examples/lab5/`. Docs: `docs/module-1-ros2/labs/lab5.md`
-   [X] T024 [P] [US5] Draft Module 1: ROS 2 Application to Humanoid Robotics. docs/module-1-ros2/index.md
-   [X] T025 [P] [US5] Draft Module 1: ROS 2 Debugging & Troubleshooting (including specific error scenarios and observability). docs/module-1-ros2/index.md
-   [X] T026 [P] [US5] Draft Module 1: ROS 2 Assessment Criteria. docs/module-1-ros2/index.md
-   [X] T027 [P] [US5] Draft Module 1: ROS 2 Summary. docs/module-1-ros2/index.md
-   [X] T028 [P] [US5] Draft Module 1: ROS 2 Further Reading (APA Style). docs/module-1-ros2/index.md

**Checkpoint**: Module 1 content is complete, functional, and testable independently.

---

## Phase 4: Module 2 - The Digital Twin — Gazebo & Unity Simulation Environments

**Goal**: Develop content for the Digital Twin chapter, covering Gazebo and Unity simulation, sensor modeling, and establishing a Gazebo ↔ ROS ↔ Unity pipeline.

**Independent Test**: Students can successfully set up Gazebo, load humanoid URDF/SDF models, integrate simulated sensors, and establish a functional two-way communication pipeline between Gazebo, ROS, and Unity, verifying all functional requirements and labs are reproducible.

### Implementation for Module 2

-   [X] T029 [P] [US1] Draft Module 2: Digital Twin Overview and Purpose. docs/module-2-digital-twin/index.md
-   [X] T030 [P] [US1] Draft Module 2: What is a Digital Twin?. docs/module-2-digital-twin/index.md
-   [X] T031 [P] [US1] Draft Module 2: Gazebo Physics and Humanoid Simulation. docs/module-2-digital-twin/index.md
-   [X] T032 [P] [US1] Draft Module 2: Sensor Simulation. docs/module-2-digital-twin/index.md
-   [X] T033 [P] [US2] Draft Module 2: Unity for High-Fidelity Visualization. docs/module-2-digital-twin/index.md
-   [X] T034 [P] [US2] Draft Module 2: Gazebo ↔ ROS ↔ Unity Pipeline. docs/module-2-digital-twin/index.md
-   [X] T035 [P] [US2] Implement Lab 1: Set up Gazebo on Ubuntu 22.04. Docs: `docs/module-2-digital-twin/labs/lab1.md`
-   [X] T036 [P] [US3] Implement Lab 2: Load humanoid URDF in Gazebo. Docs: `docs/module-2-digital-twin/labs/lab2.md`
-   [X] T037 [P] [US4] Implement Lab 3: Add LiDAR + IMU + Depth camera. Docs: `docs/module-2-digital-twin/labs/lab3.md`
-   [X] T038 [P] [US5] Implement Lab 4: Export environment to Unity. Docs: `docs/module-2-digital-twin/labs/lab4.md`
-   [X] T039 [P] [US5] Implement Lab 5: Create a Digital Twin scene with two-way ROS communication. Docs: `docs/module-2-digital-twin/labs/lab5.md`
-   [X] T040 [P] [US5] Draft Module 2: Digital Twin Troubleshooting. docs/module-2-digital-twin/index.md
-   [X] T041 [P] [US5] Draft Module 2: Digital Twin Summary. docs/module-2-digital-twin/index.md
-   [X] T042 [P] [US5] Draft Module 2: Digital Twin Further Reading. docs/module-2-digital-twin/index.md

**Checkpoint**: Module 2 content is complete, functional, and testable independently.

---

## Phase 5: Module 3 - The AI-Robot Brain — NVIDIA Isaac Simulation and Isaac ROS

**Goal**: Develop content for the NVIDIA Isaac Sim and Isaac ROS chapter, explaining perception, VSLAM, navigation, synthetic data, and Sim-to-Real transfer.

**Independent Test**: Students can successfully install and launch Isaac Sim, build and run Isaac ROS perception and VSLAM pipelines, create a Nav2 navigation demo, generate synthetic data, and deploy ROS nodes to a Jetson Orin, verifying all functional requirements and labs are reproducible.

### Implementation for Module 3

-   [X] T043 [P] [US1] Draft Module 3: Isaac Sim Overview. docs/module-3-isaac-sim-ros/index.md
-   [X] T044 [P] [US1] Draft Module 3: Isaac Sim Architecture (USD scenes, RTX rendering). docs/module-3-isaac-sim-ros/index.md
-   [X] T045 [P] [US1] Draft Module 3: Isaac ROS Perception Pipeline. docs/module-3-isaac-sim-ros/index.md
-   [X] T046 [P] [US1] Draft Module 3: Navigation with Nav2. docs/module-3-isaac-sim-ros/index.md
-   [X] T047 [P] [US2] Draft Module 3: VSLAM (Visual SLAM). docs/module-3-isaac-sim-ros/index.md
-   [X] T048 [P] [US2] Draft Module 3: Photorealistic Rendering + Sensors. docs/module-3-isaac-sim-ros/index.md
-   [X] T049 [P] [US2] Draft Module 3: Synthetic Data Workflows. docs/module-3-isaac-sim-ros/index.md
-   [X] T050 [P] [US3] Draft Module 3: Sim-to-Real Techniques. docs/module-3-isaac-sim-ros/index.md
-   [X] T051 [P] [US3] Draft Module 3: Jetson Deployment. docs/module-3-isaac-sim-ros/index.md
-   [X] T052 [P] [US4] Implement Lab 1: Install Isaac Sim on RTX Workstation. Docs: `docs/module-3-isaac-sim-ros/labs/lab1.md`
-   [X] T053 [P] [US4] Implement Lab 2: Build a perception pipeline. Docs: `docs/module-3-isaac-sim-ros/labs/lab2.md`
-   [X] T054 [P] [US5] Implement Lab 3: Run Isaac ROS Visual SLAM. Docs: `docs/module-3-isaac-sim-ros/labs/lab3.md`
-   [X] T055 [P] [US5] Implement Lab 4: Create a Nav2 navigation demo. Docs: `docs/module-3-isaac-sim-ros/labs/lab4.md`
-   [X] T056 [P] [US6] Implement Lab 5: Generate synthetic images for training. Docs: `docs/module-3-isaac-sim-ros/labs/lab5.md`
-   [X] T057 [P] [US6] Implement Lab 6: Deploy ROS nodes to Jetson Orin. Docs: `docs/module-3-isaac-sim-ros/labs/lab6.md`
-   [X] T058 [P] [US6] Draft Module 3: Isaac Sim Troubleshooting. docs/module-3-isaac-sim-ros/index.md
-   [X] T059 [P] [US6] Draft Module 3: Isaac Sim Summary. docs/module-3-isaac-sim-ros/index.md
-   [X] T060 [P] [US6] Draft Module 3: Isaac Sim Further Reading. docs/module-3-isaac-sim-ros/index.md

**Checkpoint**: Module 3 content is complete, functional, and testable independently.

---

## Phase 6: Module 4 - Vision-Language-Action (VLA) for Humanoid Robotics

**Goal**: Develop content for the VLA chapter, covering voice commands, LLM reasoning, object detection, and action generation for humanoid robots.

**Independent Test**: Students can successfully implement voice-to-text conversion using Whisper, use an LLM for text-to-action plan generation, perform object detection, and enable a robot to perform basic manipulation tasks based on verbal instructions, verifying all functional requirements and labs are reproducible.

### Implementation for Module 4

-   [X] T061 [P] [US1] Draft Module 4: Introduction to VLA. docs/module-4-vla/index.md
-   [X] T062 [P] [US1] Draft Module 4: Voice Recognition Pipeline (Whisper). docs/module-4-vla/index.md
-   [X] T063 [P] [US1] Draft Module 4: Natural Language → Robot Planning. docs/module-4-vla/index.md
-   [X] T064 [P] [US1] Draft Module 4: Vision for Object Recognition. docs/module-4-vla/index.md
-   [X] T065 [P] [US2] Draft Module 4: Action Execution using ROS 2. docs/module-4-vla/index.md
-   [X] T066 [P] [US2] Draft Module 4: Full VLA Pipeline Architecture. docs/module-4-vla/index.md
-   [X] T067 [P] [US2] Implement Lab 1: Whisper installation + voice command, code in `src/vla_agents/lab1/`. Docs: `docs/module-4-vla/labs/lab1.md`
-   [X] T068 [P] [US3] Implement Lab 2: Use LLM to convert text → ROS action plan, code in `src/vla_agents/lab2/`. Docs: `docs/module-4-vla/labs/lab2.md`
-   [X] T069 [P] [US4] Implement Lab 3: Detect objects using YOLO/Isaac ROS, code in `src/vla_agents/lab3/`. Docs: `docs/module-4-vla/labs/lab3.md`
-   [X] T070 [P] [US5] Implement Lab 4: Robot picks an object after verbal instruction, code in `src/vla_agents/lab4/`. Docs: `docs/module-4-vla/labs/lab4.md`
-   [X] T071 [P] [US5] Implement Lab 5: Build final VLA project: “Clean the room” pipeline, code in `src/vla_agents/lab5/`. Docs: `docs/module-4-vla/labs/lab5.md`
-   [X] T072 [P] [US5] Draft Module 4: VLA Troubleshooting. docs/module-4-vla/index.md
-   [X] T073 [P] [US5] Draft Module 4: VLA Summary. docs/module-4-vla/index.md
-   [X] T074 [P] [US5] Draft Module 4: VLA Further Reading. docs/module-4-vla/index.md

**Checkpoint**: Module 4 content is complete, functional, and testable independently.

---

## Phase 7: Synthesis & Finalization

**Purpose**: Integrating all modules, ensuring cross-book consistency, final validation, and preparing for deployment.

-   [X] T075 Draft Capstone Project Specification for `docs/capstone-project.md`.
-   [X] T076 Integrate all module labs into coherent Hands-On Tutorials for `docs/lab-guides.md`.
-   [ ] T077 Ensure cross-module consistency (terminology, examples, style) across `docs/`.
-   [ ] T078 Validate all code and simulation steps across all modules for correctness and reproducibility in `src/`.
-   [ ] T079 Finalize Global Sections (Preface, Overview, Weekly Breakdown, Hardware Appendix, Glossary, Sim-to-Real) in `docs/`.
-   [ ] T080 Final Docusaurus build and deployment to GitHub Pages validation.

---

## Architectural Decision Record (ADR) Tasks

**Purpose**: Documenting key architectural decisions identified during planning.

-   [ ] T081 Create ADR for Simulation Engine Choice (Gazebo vs Unity). history/adr/simulation-engine-choice.md
-   [ ] T082 Create ADR for Programming Language for Humanoid Control (Python/rclpy vs C++). history/adr/humanoid-control-language.md
-   [ ] T083 Create ADR for Perception Stack (Isaac ROS vs OpenCV/Custom Models). history/adr/perception-stack-choice.md
-   [ ] T084 Create ADR for VLA System Design (Local vs Cloud LLM). history/adr/vla-system-design.md
-   [ ] T085 Create ADR for Hardware Strategy (Physical Lab vs Cloud Sim Rig). history/adr/hardware-strategy.md

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Research (Phase 1)**: No dependencies - can start immediately.
-   **Foundation (Phase 2)**: Depends on Research completion.
-   **Module Phases (Phase 3-6)**: All depend on Foundation completion. Modules can then proceed in parallel (if staffed) or sequentially.
-   **Synthesis (Phase 7)**: Depends on all Module Phases completion.
-   **ADR Tasks**: Can be initiated as soon as a decision is made, but should ideally be completed before Synthesis.

### User Story Dependencies (within modules)

-   User stories within each module are generally independent but follow a logical progression from basic setup to advanced concepts.

### Within Each Task Group

-   Code implementation tasks within a lab depend on the lab documentation being drafted.
-   Diagrams should be created before integrating into chapter text.

### Parallel Opportunities

-   All tasks marked `[P]` can run in parallel within their respective phases.
-   Once the "Foundation" phase is complete, different modules can be developed in parallel by different team members.
-   Within each module, drafting different sections (Overview, Core Concepts, etc.) can be parallelized.
-   Implementing different labs within a module can be parallelized.
-   ADR tasks can be worked on in parallel with module development as decisions are finalized.

---

## Parallel Example: Module 1 (ROS 2) Implementation

```bash
# Research tasks can run in parallel
Task: "Gather ROS 2 primary sources for Module 1 documentation. docs/module-1-ros2/"
Task: "Build initial reference architecture diagrams for the overall book structure. static/diagrams/book_architecture.png"

# Once Foundation is done, Module 1 drafting can start
Task: "Draft Module 1: ROS 2 Overview + Learning Outcomes. docs/module-1-ros2/index.md"
Task: "Draft Module 1: ROS 2 Core Concepts. docs/module-1-ros2/index.md"
Task: "Draft Module 1: ROS 2 Architecture + Diagrams. docs/module-1-ros2/index.md"

# Lab implementations can also run in parallel
Task: "Implement Lab 1: Create a ROS 2 package, code in src/ros2_package_examples/lab1/. Docs: docs/module-1-ros2/labs/lab1.md"
Task: "Implement Lab 2: Publish and subscribe to topics, code in src/ros2_package_examples/lab2/. Docs: docs/module-1-ros2/labs/lab2.md"
Task: "Implement Lab 3: Build a humanoid URDF (torso + arms + legs), code in src/ros2_package_examples/lab3/. Docs: docs/module-1-ros2/labs/lab3.md"
```

---

## Implementation Strategy

### MVP First (Module 1 Only)

1.  Complete Phase 1: Research.
2.  Complete Phase 2: Foundation (CRITICAL - blocks all modules).
3.  Complete Phase 3: Module 1 (ROS 2) entirely.
4.  **STOP and VALIDATE**: Test Module 1 content and labs independently.
5.  Deploy/demo Module 1 if ready.

### Incremental Delivery

1.  Complete Research + Foundation → Book skeleton ready.
2.  Add Module 1 → Test independently → Deploy/Demo.
3.  Add Module 2 → Test independently → Deploy/Demo.
4.  Add Module 3 → Test independently → Deploy/Demo.
5.  Add Module 4 → Test independently → Deploy/Demo.
6.  Complete Synthesis Phase → Final book release.

### Parallel Team Strategy

With multiple developers:

1.  Team completes Research + Foundation together.
2.  Once Foundation is done:
    -   Developer A: Module 1
    -   Developer B: Module 2
    -   Developer C: Module 3
    -   Developer D: Module 4
    -   Developer E: Synthesis & ADRs
3.  Modules complete and integrate independently.

---

## Notes

-   `[P]` tasks = different files, no dependencies.
-   `[Story]` label maps task to specific user story for traceability.
-   Each user story/module should be independently completable and testable.
-   Verify code examples and simulations for correctness.
-   Commit after each task or logical group.
-   Stop at any checkpoint to validate module independently.
-   Avoid: vague tasks, same file conflicts, cross-module dependencies that break independence without clear management.
