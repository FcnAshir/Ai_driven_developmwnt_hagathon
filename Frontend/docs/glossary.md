---
sidebar_position: 10
---

# Glossary & References

## Glossary

### A

**Action (ROS 2)**: A communication pattern in ROS 2 that allows for long-running tasks with feedback and goal preemption. Actions extend the service pattern by adding feedback and the ability to cancel goals.

**Actuator**: A mechanical device that converts control signals into physical motion. In robotics, actuators control the movement of robot joints and end-effectors.

**Adaptive Control**: A control method that adjusts controller parameters in real-time based on changes in system dynamics or operating conditions.

**AI (Artificial Intelligence)**: The simulation of human intelligence processes by machines, especially computer systems. In robotics, AI enables perception, reasoning, and decision-making.

**Apriltag**: A visual fiducial system that consists of a 2D barcode-like image that can be used for camera pose estimation and robot localization.

**Autonomous System**: A system that can operate independently without human intervention, making decisions and taking actions based on its programming and environmental inputs.

### B

**Behavior Tree**: A hierarchical structure used in robotics and AI to organize and execute complex behaviors. Behavior trees are popular in game AI and robotics for their modularity and reusability.

**Bipedal Locomotion**: The act of walking on two legs, a key capability for humanoid robots that must navigate human environments.

**Bridge**: In the context of robotics simulation, a component that enables communication between simulation and real-world systems (e.g., ROS bridge for Unity).

### C

**CenterPose**: An Isaac ROS package that provides 6D object pose estimation, allowing robots to determine the position and orientation of objects in 3D space.

**Collision Detection**: The computational problem of detecting whether two or more geometric objects intersect or come into contact.

**Computer Vision**: A field of artificial intelligence that trains computers to interpret and understand the visual world, using digital images and deep learning models.

**Contact Dynamics**: The study of forces and motions that occur when objects come into contact, crucial for realistic simulation and robot manipulation.

**Control Theory**: An interdisciplinary branch of engineering and mathematics that deals with the behavior of dynamical systems with inputs, and how their behavior is modified by feedback.

**Controller**: A device or algorithm that manages and commands a mechanical system, such as a robot's actuators, to achieve desired behavior.

### D

**DDS (Data Distribution Service)**: A middleware protocol and API standard for distributed real-time systems. ROS 2 uses DDS as its communication layer.

**Deep Learning**: A subset of machine learning based on artificial neural networks with representation learning. Deep learning can model complex non-linear relationships.

**Deep Neural Network (DNN)**: An artificial neural network with multiple layers between the input and output layers, used for complex pattern recognition tasks.

**Digital Twin**: A virtual representation of a physical object or system that spans its lifecycle, is updated with real-time data, and uses simulation, machine learning, and reasoning to help decision-making.

**Domain Adaptation**: A machine learning technique that adapts models trained on one domain (e.g., simulation) to work effectively on a different but related domain (e.g., reality).

**Domain Randomization**: A technique in robotics simulation where physical parameters, visual properties, and environmental conditions are randomly varied during training to improve robustness.

### E

**Embodied AI**: Artificial intelligence that is integrated into physical systems (robots) that can interact with the real world through sensors and actuators.

**End-Effector**: The device at the end of a robot arm that interacts with the environment, such as a gripper or tool.

**Environment**: The physical or simulated space in which a robot operates, including objects, obstacles, and other agents.

### F

**Feedback Control**: A control system that uses measurements of the output to adjust the input, creating a closed-loop system that can respond to disturbances.

**Fiducial**: A visual marker with a known geometric shape that can be detected by computer vision algorithms for localization and pose estimation.

**Forward Kinematics**: The process of calculating the position and orientation of a robot's end-effector based on the joint angles of its actuators.

### G

**Gazebo**: A physics-based 3D simulation environment used for robotics development, testing, and research. It provides realistic simulation of sensors and actuators.

**Gripper**: A robotic device designed to grasp and hold objects, typically located at the end of a robot arm.

**Grounded Language Learning**: The process of learning language in the context of physical interaction with the world, connecting words to sensory experiences and actions.

### H

**Hardware-in-the-Loop (HIL)**: A testing technique that involves running a real-time simulation with actual hardware components connected to the simulation.

**Human-Robot Interaction (HRI)**: The study of interactions between humans and robots, focusing on design, development, and evaluation of robotic systems for human use.

**Humanoid Robot**: A robot with a physical structure that resembles the human body, typically having a head, torso, two arms, and two legs.

### I

**Imitation Learning**: A machine learning technique where an agent learns to perform tasks by observing and mimicking expert demonstrations.

**Inverse Kinematics**: The process of calculating the joint angles required to position a robot's end-effector at a desired location and orientation.

**Isaac ROS**: A collection of hardware-accelerated perception and navigation packages that enable developers to build perception and navigation capabilities for robots using NVIDIA GPUs.

**Isaac Sim**: NVIDIA's next-generation robotics simulation application based on NVIDIA Omniverse, providing high-fidelity simulation for robotics development.

**Integration**: In control systems, the process of summing error over time, used in PID controllers to eliminate steady-state error.

### J

**Joint**: A connection between two or more robot links that allows relative motion. Joints can be revolute (rotary), prismatic (linear), or other types.

### K

**Kinematics**: The study of motion without considering the forces that cause it. In robotics, kinematics describes the relationship between joint angles and end-effector position.

**Kinodynamic Planning**: Motion planning that considers both kinematic constraints (geometry) and dynamic constraints (physics) of the robot system.

### L

**Language Model**: An AI model designed to understand and generate human language, often used in robotics for natural language processing.

**Latency**: The time delay between a stimulus and the response, critical in real-time robotic systems where delays can cause instability.

**LiDAR**: Light Detection and Ranging, a remote sensing method that uses light in the form of a pulsed laser to measure distances.

**Locomotion**: The ability to move from one place to another, a fundamental capability for mobile robots including walking, rolling, or flying.

**Long Short-Term Memory (LSTM)**: A type of recurrent neural network designed to learn from sequences of data with long-term dependencies.

### M

**Manipulation**: The ability to purposefully change the state of objects in the environment, typically through grasping and moving objects.

**Middleware**: Software that provides common services and capabilities to applications beyond what's offered by the operating system, such as ROS 2's DDS implementation.

**Motion Planning**: The computational problem of automatically planning a path for a robot to follow from a start to a goal state while avoiding obstacles.

**MoveIt**: The standard ROS package for motion planning, manipulation, 3D perception, kinematics, control, and navigation.

### N

**Navigation**: The ability of a robot to move through an environment from one location to another, typically involving mapping, localization, and path planning.

**Navigation2**: The latest navigation stack for ROS 2, providing tools for robot path planning, execution, and recovery.

**Neural Network**: A computing system inspired by biological neural networks, used in machine learning for pattern recognition and decision making.

**Node (ROS)**: A process that performs computation in ROS. Nodes are the fundamental building blocks of ROS applications.

### O

**Object Detection**: A computer vision technique that identifies and localizes objects within images or video streams.

**Omniverse**: NVIDIA's simulation and collaboration platform for 3D design workflows, used as the foundation for Isaac Sim.

**OpenAI**: An artificial intelligence research laboratory consisting of the for-profit OpenAI LP and the non-profit OpenAI Inc.

**Operational Space Control**: A control framework that allows specifying desired motion and forces in the Cartesian space of the robot's end-effector.

### P

**Path Planning**: The computational process of finding a collision-free path from a start to a goal configuration in an environment.

**Perception**: The ability of a robot to interpret sensory information from its environment to understand and interact with the world.

**PID Controller**: A control loop mechanism employing feedback that uses Proportional, Integral, and Derivative terms to minimize error.

**Physical AI**: Artificial intelligence that is embodied and interacts with the physical world, combining perception, reasoning, and action.

**Physics Engine**: Software that simulates physical systems, providing realistic motion, collision detection, and response in simulations.

**Point Cloud**: A set of data points in space, typically produced by 3D scanning devices like LiDAR sensors.

**Pose Estimation**: The process of determining the position and orientation of an object or robot in 3D space.

### Q

**Quaternion**: A mathematical representation of rotation in 3D space, commonly used in robotics for orientation representation without gimbal lock.

### R

**Reinforcement Learning**: A type of machine learning where an agent learns to make decisions by performing actions and receiving rewards or penalties.

**Robot Operating System (ROS)**: A flexible framework for writing robot software, providing hardware abstraction, device drivers, and message passing.

**ROS 2**: The second generation of the Robot Operating System, designed for production environments with improved security and real-time capabilities.

**RRT (Rapidly-exploring Random Tree)**: A motion planning algorithm that builds a tree of possible paths by randomly exploring the configuration space.

**Rviz**: The 3D visualization tool for ROS, used for displaying robot models, sensor data, and other information in a 3D environment.

### S

**Sensor Fusion**: The process of combining data from multiple sensors to improve the accuracy and reliability of perception.

**SLAM (Simultaneous Localization and Mapping)**: The computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location.

**Simulation**: The imitation of the operation of a real-world process or system over time, used extensively in robotics development and testing.

**State Estimation**: The process of estimating the internal state of a system from noisy measurements, often using filters like Kalman filters.

**System Identification**: The process of developing mathematical models of dynamic systems from measured input-output data.

### T

**Task Planning**: The process of decomposing high-level goals into sequences of executable actions, considering available resources and constraints.

**Teleoperation**: The remote operation of a robot by a human operator, often used for tasks in dangerous or inaccessible environments.

**Trajectory**: A time-parameterized path that specifies the desired position, velocity, and acceleration of a robot over time.

**Transformer**: A type of neural network architecture that uses self-attention mechanisms, widely used in natural language processing and computer vision.

### U

**URDF (Unified Robot Description Format)**: An XML format for representing a robot model, including kinematic and dynamic properties, visual appearance, and collision properties.

**USD (Universal Scene Description)**: NVIDIA's scene description and 3D graphics interchange framework, used in Isaac Sim for scene composition.

### V

**VLA (Vision-Language-Action)**: A framework for embodied AI that integrates visual perception, natural language understanding, and physical action.

**Vision-Language Model (VLM)**: AI models that can understand and generate responses based on both visual and textual inputs.

**Virtual Reality (VR)**: A simulated experience that can be similar to or completely different from the real world, used in robotics for teleoperation and training.

### W

**Whisper**: OpenAI's automatic speech recognition system, used for converting speech to text in robotics applications.

**Whole-Body Control**: A control framework that coordinates all degrees of freedom of a robot simultaneously to achieve multiple tasks.

## References

### Primary Sources

1. ROS.org. (2024). *ROS 2 Documentation*. Retrieved from https://docs.ros.org/
2. NVIDIA. (2024). *Isaac Sim Documentation*. Retrieved from https://docs.omniverse.nvidia.com/isaacsim/latest/
3. NVIDIA. (2024). *Isaac ROS Documentation*. Retrieved from https://nvidia-isaac-ros.github.io/
4. OpenAI. (2024). *Whisper: Robust Speech Recognition via Large-Scale Weak Supervision*. arXiv preprint arXiv:2209.09867.
5. Quigley, M., et al. (2009). *ROS: an open-source Robot Operating System*. ICRA Workshop on Open Source Software, 3(3.2), 5.

### Technical Papers

6. Fox, D., Burgard, W., & Thrun, S. (1997). *The dynamic window approach to collision avoidance*. IEEE Robotics & Automation Magazine, 4(1), 23-33.
7. Khatib, O. (1986). *Real-time obstacle avoidance for manipulators and mobile robots*. The International Journal of Robotics Research, 5(1), 90-98.
8. LaValle, S. M., & Kuffner Jr, J. J. (2001). *Randomized kinodynamic planning*. The International Journal of Robotics Research, 20(5), 378-400.
9. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic robotics*. MIT press.
10. Siciliano, B., & Khatib, O. (Eds.). (2016). *Springer handbook of robotics*. Springer.

### Simulation and Digital Twin

11. Koening, N., & Howard, A. (2004). *Design and use paradigms for Gazebo, an open-source multi-robot simulator*. Proceedings of the 2004 IEEE/RSJ International Conference on Intelligent Robots and Systems, 3(2), 2149-2154.
12. NVIDIA Corporation. (2024). *Isaac Sim: NVIDIA's Next Generation Robotics Simulation Application*. Technical Report.
13. Unity Technologies. (2024). *Unity Robotics Hub: Tools and Resources for Robotics Simulation*. Technical Documentation.

### AI and Machine Learning for Robotics

14. Levine, S., et al. (2016). *End-to-end training of deep visuomotor policies*. The Journal of Machine Learning Research, 17(1), 1334-1373.
15. Pinto, L., & Gupta, A. (2016). *Supersizing self-supervision: Learning to grasp from 50k tries and 700 robot hours*. arXiv preprint arXiv:1509.06825.
16. Tobin, J., et al. (2017). *Domain randomization for transferring deep neural networks from simulation to the real world*. 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 23-30.
17. James, S., et al. (2019). *Sim-to-real via sim-to-sim: Data-efficient robotic grasping via randomized-to-canonical adaptation networks*. Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition, 12627-12636.

### Humanoid Robotics

18. Kajita, S., et al. (2003). *Biped walking pattern generation by using preview control of zero-moment point*. 2003 IEEE International Conference on Robotics and Automation, 2(2), 1620-1626.
19. Pratt, J., & Walking, M. (2006). *Virtual model control: An intuitive approach for bipedal locomotion*. The International Journal of Robotics Research, 20(2), 293-302.
20. Englsberger, J., et al. (2015). *Three-dimensional bipedal walking control using Divergent Component of Motion*. 2015 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 1987-1993.

### Vision-Language-Action Systems

21. Zhu, Y., et al. (2017). *Target-driven visual navigation in indoor scenes using deep reinforcement learning*. 2017 IEEE International Conference on Robotics and Automation (ICRA), 3357-3364.
22. Misra, D., et al. (2019). *Mapping natural language instructions to mobile robot actions*. arXiv preprint arXiv:1902.08515.
23. Huang, W., et al. (2022). *Collaborative grounding of large language models for robotic manipulation*. arXiv preprint arXiv:2210.01911.
24. Brohan, C., et al. (2022). *RT-1: Robotics transformer for real-world control at scale*. arXiv preprint arXiv:2208.01876.

### Isaac ROS Packages

25. NVIDIA. (2024). *Isaac ROS Apriltag: High-Performance Fiducial Detection*. Technical Documentation.
26. NVIDIA. (2024). *Isaac ROS Visual SLAM: Hardware-Accelerated Visual Simultaneous Localization and Mapping*. Technical Documentation.
27. NVIDIA. (2024). *Isaac ROS CenterPose: 6D Object Pose Estimation*. Technical Documentation.

### Standards and Best Practices

28. ISO 13482:2014. *Robots and robotic devices — Safety requirements for personal care robots*. International Organization for Standardization.
29. Murphy, R. R. (2014). *Introduction to AI robotics*. MIT press.
30. Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot modeling and control*. John Wiley & Sons.

### Software and Tools

31. Colas, F., et al. (2020). *A practical guide to ROS 2 for researchers in robotics and AI*. arXiv preprint arXiv:2006.09447.
32. Dube, R., et al. (2020). *SegMap: 3D Segment Mapping using Deep Neural Networks*. 2020 IEEE International Conference on Robotics and Automation (ICRA).
33. Quigley, M., et al. (2009). *RViz: An Extensible Visualization Tool for Robot Data Display*. ICRA Workshop on Open Source Software.

This glossary and reference list will be continuously updated throughout the course development to ensure comprehensive coverage of all concepts and proper attribution of sources used in the book.