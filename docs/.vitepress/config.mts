import { defineConfig } from 'vitepress'
import { withMermaid } from "vitepress-plugin-mermaid";

// https://vitepress.dev/reference/site-config
export default withMermaid({
  title: "OpenMowerROS2",
  description: "Just another software stack for OpenMower lawn mower robot, but this time with ROS2!",
  themeConfig: {
    // https://vitepress.dev/reference/default-theme-config
    nav: [
      { text: 'Home', link: '/' },
      { text: 'Documentation', link: '/getting-started' }
    ],

    sidebar: [
      {
        text: 'Getting started',
        link: '/getting-started',
        // items: [
        //   { text: 'Markdown Examples', link: '/markdown-examples' },
        //   { text: 'Runtime API Examples', link: '/api-examples' }
        // ]
      },
      {
        text: 'Components explained',
        items: [
          { text: 'ROS2 workspace', link: '/ros2-workspace' },
          { text: 'Mainboard firmware', link: '/omros2-firmware' },
        ]
      },
      {
        text: 'Contributing guide',
        link: '/contributing',
        items: [
            { text: 'Devcontainer', link: '/contributing-devcontainer' },
            { text: 'Simulation', link: '/contributing-simulation' },
        ],
      },
    ],

    socialLinks: [
      { icon: 'github', link: 'https://github.com/jkaflik/OpenMowerROS2' },
      { icon: 'discord', link: 'https://discord.gg/jE7QNaSxW7' },
    ]
  }
})
