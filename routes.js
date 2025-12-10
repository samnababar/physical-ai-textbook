import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/markdown-page',
    component: ComponentCreator('/markdown-page', '3d7'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'f2c'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '5cc'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'cb7'),
            routes: [
              {
                path: '/docs/ch1-ros2/week-1',
                component: ComponentCreator('/docs/ch1-ros2/week-1', '026'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ch1-ros2/week-2',
                component: ComponentCreator('/docs/ch1-ros2/week-2', 'd88'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ch1-ros2/week-3',
                component: ComponentCreator('/docs/ch1-ros2/week-3', 'bc5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ch1-ros2/week-4',
                component: ComponentCreator('/docs/ch1-ros2/week-4', '964'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ch2-digital-twin/week-1',
                component: ComponentCreator('/docs/ch2-digital-twin/week-1', 'e1c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ch2-digital-twin/week-2',
                component: ComponentCreator('/docs/ch2-digital-twin/week-2', 'dfa'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ch2-digital-twin/week-3',
                component: ComponentCreator('/docs/ch2-digital-twin/week-3', '651'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ch2-digital-twin/week-4',
                component: ComponentCreator('/docs/ch2-digital-twin/week-4', 'e95'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ch3-nvidia-isaac/week-1',
                component: ComponentCreator('/docs/ch3-nvidia-isaac/week-1', 'e45'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ch3-nvidia-isaac/week-2',
                component: ComponentCreator('/docs/ch3-nvidia-isaac/week-2', '474'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ch3-nvidia-isaac/week-3',
                component: ComponentCreator('/docs/ch3-nvidia-isaac/week-3', 'ad1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ch3-nvidia-isaac/week-4',
                component: ComponentCreator('/docs/ch3-nvidia-isaac/week-4', '73f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ch4-vla/week-1',
                component: ComponentCreator('/docs/ch4-vla/week-1', 'f02'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ch4-vla/week-2',
                component: ComponentCreator('/docs/ch4-vla/week-2', 'a69'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ch4-vla/week-3',
                component: ComponentCreator('/docs/ch4-vla/week-3', '461'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ch4-vla/week-4',
                component: ComponentCreator('/docs/ch4-vla/week-4', 'bd1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', 'e5f'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
