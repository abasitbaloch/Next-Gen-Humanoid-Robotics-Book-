import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/next-gen-humanoid-robotics-book/__docusaurus/debug',
    component: ComponentCreator('/next-gen-humanoid-robotics-book/__docusaurus/debug', 'b30'),
    exact: true
  },
  {
    path: '/next-gen-humanoid-robotics-book/__docusaurus/debug/config',
    component: ComponentCreator('/next-gen-humanoid-robotics-book/__docusaurus/debug/config', '3b8'),
    exact: true
  },
  {
    path: '/next-gen-humanoid-robotics-book/__docusaurus/debug/content',
    component: ComponentCreator('/next-gen-humanoid-robotics-book/__docusaurus/debug/content', '7b4'),
    exact: true
  },
  {
    path: '/next-gen-humanoid-robotics-book/__docusaurus/debug/globalData',
    component: ComponentCreator('/next-gen-humanoid-robotics-book/__docusaurus/debug/globalData', '60a'),
    exact: true
  },
  {
    path: '/next-gen-humanoid-robotics-book/__docusaurus/debug/metadata',
    component: ComponentCreator('/next-gen-humanoid-robotics-book/__docusaurus/debug/metadata', 'fc0'),
    exact: true
  },
  {
    path: '/next-gen-humanoid-robotics-book/__docusaurus/debug/registry',
    component: ComponentCreator('/next-gen-humanoid-robotics-book/__docusaurus/debug/registry', 'dec'),
    exact: true
  },
  {
    path: '/next-gen-humanoid-robotics-book/__docusaurus/debug/routes',
    component: ComponentCreator('/next-gen-humanoid-robotics-book/__docusaurus/debug/routes', 'e33'),
    exact: true
  },
  {
    path: '/next-gen-humanoid-robotics-book/docs',
    component: ComponentCreator('/next-gen-humanoid-robotics-book/docs', 'd28'),
    routes: [
      {
        path: '/next-gen-humanoid-robotics-book/docs',
        component: ComponentCreator('/next-gen-humanoid-robotics-book/docs', 'f76'),
        routes: [
          {
            path: '/next-gen-humanoid-robotics-book/docs',
            component: ComponentCreator('/next-gen-humanoid-robotics-book/docs', 'bca'),
            routes: [
              {
                path: '/next-gen-humanoid-robotics-book/docs/capstone/integration',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/capstone/integration', 'aa9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/capstone/intro',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/capstone/intro', 'c3d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/capstone/validation',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/capstone/validation', 'ce4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/intro',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/intro', '450'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-1-ros2/exercises',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-1-ros2/exercises', '227'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-1-ros2/humanoid-architectures',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-1-ros2/humanoid-architectures', '27d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-1-ros2/intro',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-1-ros2/intro', 'd7e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-1-ros2/perception-action-loops',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-1-ros2/perception-action-loops', 'e99'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-1-ros2/physical-vs-digital-ai',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-1-ros2/physical-vs-digital-ai', '60b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-1-ros2/robot-learning',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-1-ros2/robot-learning', 'c82'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-1-ros2/sim-to-real',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-1-ros2/sim-to-real', '206'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-1-ros2/validation',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-1-ros2/validation', '582'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-2-digital-twin/environments',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-2-digital-twin/environments', '977'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-2-digital-twin/intro',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-2-digital-twin/intro', '3b8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-2-digital-twin/robot-models',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-2-digital-twin/robot-models', '432'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-2-digital-twin/sensors-physics',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-2-digital-twin/sensors-physics', '11f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-2-digital-twin/testing-validation',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-2-digital-twin/testing-validation', '9bd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-2-digital-twin/unity-alternative',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-2-digital-twin/unity-alternative', '773'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-3-ai-brain/intro',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-3-ai-brain/intro', 'ac0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-3-ai-brain/manipulation-control',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-3-ai-brain/manipulation-control', '564'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-3-ai-brain/navigation-planning',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-3-ai-brain/navigation-planning', 'b98'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-3-ai-brain/perception-systems',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-3-ai-brain/perception-systems', '1a6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-3-ai-brain/sim-to-real',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-3-ai-brain/sim-to-real', '2f9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-3-ai-brain/synthetic-data',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-3-ai-brain/synthetic-data', '9e7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-4-vla/autonomous-behaviors',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-4-vla/autonomous-behaviors', 'f25'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-4-vla/human-robot-interaction',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-4-vla/human-robot-interaction', '5d6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-4-vla/intro',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-4-vla/intro', '53c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-4-vla/llm-integration',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-4-vla/llm-integration', 'a5e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-4-vla/task-decomposition',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-4-vla/task-decomposition', '3ff'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/module-4-vla/voice-recognition',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/module-4-vla/voice-recognition', '93f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/next-gen-humanoid-robotics-book/docs/setup',
                component: ComponentCreator('/next-gen-humanoid-robotics-book/docs/setup', '1b6'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/next-gen-humanoid-robotics-book/',
    component: ComponentCreator('/next-gen-humanoid-robotics-book/', '016'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
