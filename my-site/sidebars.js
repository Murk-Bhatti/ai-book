// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Module 1: ROS 2 Nervous System sidebar
  module1Sidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      collapsed: false,
      items: [
        'module-1-ros2/introduction',
        'module-1-ros2/chapter-1-ros2-nervous-system',
        'module-1-ros2/chapter-2-node-communication',
        'module-1-ros2/chapter-3-python-ai-agents',
        'module-1-ros2/chapter-4-urdf-files',
      ],
    },
  ],
  // Module 2: Digital Twin sidebar
  module2Sidebar: [
    {
      type: 'category',
      label: 'Module 2: The Digital Twin',
      collapsed: false,
      items: [
        'module-2-digital-twin/introduction',
        'module-2-digital-twin/chapter-1-digital-twins',
        'module-2-digital-twin/chapter-2-gazebo-physics',
        'module-2-digital-twin/chapter-3-unity-rendering',
        'module-2-digital-twin/chapter-4-sensor-simulation',
      ],
    },
  ],
  // Module 3: NVIDIA Isaac sidebar
  module3Sidebar: [
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain',
      collapsed: false,
      items: [
        'module-3-nvidia-isaac/introduction',
        'module-3-nvidia-isaac/chapter-1-ai-robot-brain',
        'module-3-nvidia-isaac/chapter-2-isaac-sim',
        'module-3-nvidia-isaac/chapter-3-isaac-ros',
        'module-3-nvidia-isaac/chapter-4-nav2-navigation',
      ],
    },
  ],
  // Module 4: Vision-Language-Action sidebar
  module4Sidebar: [
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      collapsed: false,
      items: [
        'module-4-vla/introduction',
        'module-4-vla/chapter-1-vla-systems',
        'module-4-vla/chapter-2-voice-to-action',
        'module-4-vla/chapter-3-llm-planning',
        'module-4-vla/chapter-4-capstone',
      ],
    },
  ],
};

export default sidebars;
