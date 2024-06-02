import { defineConfig } from 'vitepress'
import { withMermaid } from "vitepress-plugin-mermaid";

// https://vitepress.dev/reference/site-config
export default withMermaid({
  title: "OpenMowerNext",
  description: "Just another software stack for OpenMower lawn mower robot, but this time with ROS2!",
  base: "/OpenMowerNext/",
  ignoreDeadLinks: [
    // ignore all localhost links
    /^https?:\/\/localhost/,
  ],
  themeConfig: {
    search: {
      provider: 'local',
    },
    // https://vitepress.dev/reference/default-theme-config
    nav: [
      { text: 'Home', link: '/' },
      { text: 'Documentation', link: '/getting-started' }
    ],

    sidebar: [
      {
        text: 'Getting started',
        link: '/getting-started',
        items: [
          { text: 'Roadmap', link: '/roadmap' },
        ],
      },
      {
        text: 'Components explained',
        items: [
          { text: 'ROS workspace', link: '/ros-workspace' },
          { text: 'Mainboard firmware', link: '/omros2-firmware' },
          { text: 'Robot localization', link: '/localization' },
          { text: 'Map management', link: '/map-management' },
        ]
      },
      {
        text: 'Contributing guide',
        link: '/contributing',
        items: [
            {
              text: 'Devcontainer',
              link: '/devcontainer',
              items: [
                { text: 'CLion alternative', link: '/clion-env' },
              ],
            },
            { text: 'Visualisation', link: '/visualisation' },
            { text: 'Simulator', link: '/simulator' },
        ],
      },
    ],

    socialLinks: [
      { icon: 'github', link: 'https://github.com/jkaflik/OpenMowerNext' },
      { icon: 'discord', link: 'https://discord.gg/jE7QNaSxW7' },
    ]
  }
})
