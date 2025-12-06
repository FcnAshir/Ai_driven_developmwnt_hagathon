// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1 - The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2/index',
        'module-1-ros2/ros2-research',
        {
          type: 'category',
          label: 'Labs',
          items: [
            'module-1-ros2/labs/lab1',
            'module-1-ros2/labs/lab2',
            'module-1-ros2/labs/lab3',
            'module-1-ros2/labs/lab4',
            'module-1-ros2/labs/lab5'
          ]
        }
      ],
    },
    {
      type: 'category',
      label: 'Module 2 - The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/index',
        'module-2-digital-twin/gazebo-unity-research',
        {
          type: 'category',
          label: 'Labs',
          items: [
            'module-2-digital-twin/labs/lab1',
            'module-2-digital-twin/labs/lab2',
            'module-2-digital-twin/labs/lab3',
            'module-2-digital-twin/labs/lab4',
            'module-2-digital-twin/labs/lab5'
          ]
        }
      ],
    },
    {
      type: 'category',
      label: 'Module 3 - The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3-isaac-sim-ros/index',
        'module-3-isaac-sim-ros/isaac-research',
        {
          type: 'category',
          label: 'Labs',
          items: [
            'module-3-isaac-sim-ros/labs/lab1',
            'module-3-isaac-sim-ros/labs/lab2',
            'module-3-isaac-sim-ros/labs/lab3',
            'module-3-isaac-sim-ros/labs/lab4',
            'module-3-isaac-sim-ros/labs/lab5',
            'module-3-isaac-sim-ros/labs/lab6'
          ]
        }
      ],
    },
    {
      type: 'category',
      label: 'Module 4 - Vision-Language-Action (VLA)',
      items: [
        'module-4-vla/index',
        'module-4-vla/vla-research',
        {
          type: 'category',
          label: 'Labs',
          items: [
            'module-4-vla/labs/lab1',
            'module-4-vla/labs/lab2',
            'module-4-vla/labs/lab3',
            'module-4-vla/labs/lab4',
            'module-4-vla/labs/lab5'
          ]
        }
      ],
    },
    'physical-ai-overview',
    'capstone-project',
    'weekly-breakdown',
    'hardware-appendix',
    'sim-to-real',
    'glossary',
  ],
};

module.exports = sidebars;