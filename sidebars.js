// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: Physical AI Foundations',
      items: [
        'module-1-ros2/intro',
        'module-1-ros2/perception-action-loops',
        'module-1-ros2/sim-to-real',
        'module-1-ros2/humanoid-architectures',
        'module-1-ros2/physical-vs-digital-ai',
        'module-1-ros2/robot-learning',
        'module-1-ros2/exercises',
        'module-1-ros2/validation',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/intro',
        'module-2-digital-twin/robot-models',
        'module-2-digital-twin/sensors-physics',
        'module-2-digital-twin/environments',
        'module-2-digital-twin/testing-validation',
        'module-2-digital-twin/unity-alternative',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3-ai-brain/intro',
        'module-3-ai-brain/perception-systems',
        'module-3-ai-brain/navigation-planning',
        'module-3-ai-brain/manipulation-control',
        'module-3-ai-brain/synthetic-data',
        'module-3-ai-brain/sim-to-real',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla/intro',
        'module-4-vla/voice-recognition',
        'module-4-vla/llm-integration',
        'module-4-vla/task-decomposition',
        'module-4-vla/autonomous-behaviors',
        'module-4-vla/human-robot-interaction',
      ],
    },
    {
      type: 'category',
      label: 'Capstone: Autonomous Humanoid',
      items: [
        'capstone/intro',
        'capstone/integration',
        'capstone/validation',
      ],
    },
  ],
};

export default sidebars;

