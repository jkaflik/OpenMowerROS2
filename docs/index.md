---
# https://vitepress.dev/reference/default-theme-home-page
layout: home

hero:
  name: "OpenMowerROS2"
  tagline: Just another software stack for OpenMower lawn mower robot, but this time with ROS2!
  actions:
    - theme: brand
      text: Getting started
      link: /getting-started
    - theme: alt
      text: View on GitHub
      link: https://github.com/jkaflik/OpenMowerROS2
  image:
    src: ./assets/logo.png
    alt: OpenMowerROS2
---
<style>
:root {
  --vp-home-hero-name-color: transparent;
  --vp-home-hero-name-background: -webkit-linear-gradient(120deg, #0b4d04 30%, #04b958);

  --vp-home-hero-image-background-image: linear-gradient(-45deg, #898c89 50%, #898c89 50%);
  --vp-home-hero-image-filter: blur(40px);
}

@media (min-width: 640px) {
  :root {
    --vp-home-hero-image-filter: blur(56px);
  }
}

@media (min-width: 960px) {
  :root {
    --vp-home-hero-image-filter: blur(256px);
  }
}
</style>
