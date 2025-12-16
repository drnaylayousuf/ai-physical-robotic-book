import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '0ce'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', 'dd5'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'd9f'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', '68f'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '3f0'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', 'c47'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '108'),
    exact: true
  },
  {
    path: '/dashboard',
    component: ComponentCreator('/dashboard', 'c07'),
    exact: true
  },
  {
    path: '/onboarding',
    component: ComponentCreator('/onboarding', 'd54'),
    exact: true
  },
  {
    path: '/profile',
    component: ComponentCreator('/profile', '55e'),
    exact: true
  },
  {
    path: '/signin',
    component: ComponentCreator('/signin', 'b81'),
    exact: true
  },
  {
    path: '/signup',
    component: ComponentCreator('/signup', 'b30'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '465'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'd1f'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '41d'),
            routes: [
              {
                path: '/docs/',
                component: ComponentCreator('/docs/', 'b00'),
                exact: true
              },
              {
                path: '/docs/',
                component: ComponentCreator('/docs/', 'df4'),
                exact: true
              },
              {
                path: '/docs/api-reference',
                component: ComponentCreator('/docs/api-reference', '026'),
                exact: true
              },
              {
                path: '/docs/capstone/capstone-overview',
                component: ComponentCreator('/docs/capstone/capstone-overview', '950'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/capstone/capstone-pipeline',
                component: ComponentCreator('/docs/capstone/capstone-pipeline', '72d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/capstone/final-demo',
                component: ComponentCreator('/docs/capstone/final-demo', '6af'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/capstone/system-architecture',
                component: ComponentCreator('/docs/capstone/system-architecture', 'c67'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chatbot-floating-widget',
                component: ComponentCreator('/docs/chatbot-floating-widget', 'aa1'),
                exact: true
              },
              {
                path: '/docs/conversational-robotics/gpt-robots',
                component: ComponentCreator('/docs/conversational-robotics/gpt-robots', 'bab'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/conversational-robotics/multimodal-interaction',
                component: ComponentCreator('/docs/conversational-robotics/multimodal-interaction', '19c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/conversational-robotics/speech-recognition',
                component: ComponentCreator('/docs/conversational-robotics/speech-recognition', 'ac1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/glossary',
                component: ComponentCreator('/docs/glossary', '65f'),
                exact: true
              },
              {
                path: '/docs/humanoid/bipedal-locomotion',
                component: ComponentCreator('/docs/humanoid/bipedal-locomotion', '046'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/humanoid/interaction-design',
                component: ComponentCreator('/docs/humanoid/interaction-design', '56a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/humanoid/manipulation',
                component: ComponentCreator('/docs/humanoid/manipulation', '0ac'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro/learning-outcomes',
                component: ComponentCreator('/docs/intro/learning-outcomes', '7ec'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro/overview',
                component: ComponentCreator('/docs/intro/overview', '304'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro/why-physical-ai-matters',
                component: ComponentCreator('/docs/intro/why-physical-ai-matters', 'a8d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/lab/cloud-vs-onprem',
                component: ComponentCreator('/docs/lab/cloud-vs-onprem', '784'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/lab/edge-kits',
                component: ComponentCreator('/docs/lab/edge-kits', '816'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/lab/hardware',
                component: ComponentCreator('/docs/lab/hardware', 'ffa'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/lab/latency-trap',
                component: ComponentCreator('/docs/lab/latency-trap', 'e08'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/lab/robot-options',
                component: ComponentCreator('/docs/lab/robot-options', '131'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module1-ros2/intro',
                component: ComponentCreator('/docs/modules/module1-ros2/intro', '5a3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module1-ros2/nodes-topics-services',
                component: ComponentCreator('/docs/modules/module1-ros2/nodes-topics-services', 'f23'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module1-ros2/ros2-architecture',
                component: ComponentCreator('/docs/modules/module1-ros2/ros2-architecture', 'f70'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module1-ros2/urdf-humanoids',
                component: ComponentCreator('/docs/modules/module1-ros2/urdf-humanoids', '4f9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module2-simulation/gazebo-physics',
                component: ComponentCreator('/docs/modules/module2-simulation/gazebo-physics', '810'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module2-simulation/intro',
                component: ComponentCreator('/docs/modules/module2-simulation/intro', 'a9a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module2-simulation/sensor-simulation',
                component: ComponentCreator('/docs/modules/module2-simulation/sensor-simulation', '7e2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module2-simulation/unity-visualization',
                component: ComponentCreator('/docs/modules/module2-simulation/unity-visualization', '470'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module3-isaac/isaac-intro',
                component: ComponentCreator('/docs/modules/module3-isaac/isaac-intro', '368'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module3-isaac/isaac-ros',
                component: ComponentCreator('/docs/modules/module3-isaac/isaac-ros', '860'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module3-isaac/isaac-sim',
                component: ComponentCreator('/docs/modules/module3-isaac/isaac-sim', '927'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module3-isaac/nav2-humanoids',
                component: ComponentCreator('/docs/modules/module3-isaac/nav2-humanoids', 'b1c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module4-vla/llm-planning',
                component: ComponentCreator('/docs/modules/module4-vla/llm-planning', '4ee'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module4-vla/multimodal-robotics',
                component: ComponentCreator('/docs/modules/module4-vla/multimodal-robotics', '3a8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module4-vla/vla-intro',
                component: ComponentCreator('/docs/modules/module4-vla/vla-intro', 'e40'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/modules/module4-vla/whisper-voice-to-action',
                component: ComponentCreator('/docs/modules/module4-vla/whisper-voice-to-action', 'f06'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/physical-ai',
                component: ComponentCreator('/docs/physical-ai', '757'),
                exact: true
              },
              {
                path: '/docs/resources',
                component: ComponentCreator('/docs/resources', '5c1'),
                exact: true
              },
              {
                path: '/docs/ros2',
                component: ComponentCreator('/docs/ros2', '39f'),
                exact: true
              },
              {
                path: '/docs/ros2/actions',
                component: ComponentCreator('/docs/ros2/actions', 'b0d'),
                exact: true
              },
              {
                path: '/docs/ros2/nodes',
                component: ComponentCreator('/docs/ros2/nodes', '987'),
                exact: true
              },
              {
                path: '/docs/ros2/services',
                component: ComponentCreator('/docs/ros2/services', 'f1a'),
                exact: true
              },
              {
                path: '/docs/ros2/topics',
                component: ComponentCreator('/docs/ros2/topics', 'a06'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', 'c73'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
