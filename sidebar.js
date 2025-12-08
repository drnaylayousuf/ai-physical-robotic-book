module.exports = {
  tutorialSidebar: [
    'intro/overview',
    'intro/why-physical-ai-matters',
    'intro/learning-outcomes',
    {
      type: 'category',
      label: 'Modules',
      items: [
        {
          type: 'category',
          label: 'Module 1 - ROS 2 Fundamentals',
          items: [
            'modules/module1-ros2/intro',
            'modules/module1-ros2/ros2-architecture',
            'modules/module1-ros2/nodes-topics-services',
            'modules/module1-ros2/urdf-humanoids',
          ],
        },
        {
          type: 'category',
          label: 'Module 2 - Simulation',
          items: [
            'modules/module2-simulation/intro',
            'modules/module2-simulation/gazebo-physics',
            'modules/module2-simulation/unity-visualization',
            'modules/module2-simulation/sensor-simulation',
          ],
        },
        {
          type: 'category',
          label: 'Module 3 - NVIDIA Isaac Platform',
          items: [
            'modules/module3-isaac/isaac-intro',
            'modules/module3-isaac/isaac-sim',
            'modules/module3-isaac/isaac-ros',
            'modules/module3-isaac/nav2-humanoids',
          ],
        },
        {
          type: 'category',
          label: 'Module 4 - Vision-Language-Action Robotics (LLM-Driven Robots)',
          items: [
            'modules/module4-vla/vla-intro',
            'modules/module4-vla/whisper-voice-to-action',
            'modules/module4-vla/llm-planning',
            'modules/module4-vla/multimodal-robotics',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Humanoid Robotics',
      items: [
        'humanoid/bipedal-locomotion',
        'humanoid/manipulation',
        'humanoid/interaction-design',
      ],
    },
    {
      type: 'category',
      label: 'Conversational Robotics',
      items: [
        'conversational-robotics/gpt-robots',
        'conversational-robotics/speech-recognition',
        'conversational-robotics/multimodal-interaction',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/capstone-overview',
        'capstone/system-architecture',
        'capstone/capstone-pipeline',
        'capstone/final-demo',
      ],
    },
    {
      type: 'category',
      label: 'Lab Setup',
      items: [
        'lab/hardware',
        'lab/edge-kits',
        'lab/robot-options',
        'lab/cloud-vs-onprem',
        'lab/latency-trap',
      ],
    },
  ],
};