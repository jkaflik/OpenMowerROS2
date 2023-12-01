---
title: Working with GRoot
---

# {{ $frontmatter.title }}

## Overview

GRoot is a GUI for [BehaviourTree.CPP](https://www.behaviortree.dev/). It is a great tool for debugging and visualizing behaviour trees.

![GRoot](https://raw.githubusercontent.com/BehaviorTree/Groot/master/groot-screenshot.png)

::: warning
BehaviourTree.CPP is not yet implemented in this project. See [issue](https://github.com/jkaflik/OpenMowerROS2/issues/9).
:::

## Run GRoot

Project comes with a prebuilt GRoot image and dedicated docker compose declaration. It does not run by default. To run it, use the following command:

```bash
$ cd .devcontainer && docker compose up groot
```

You can access it by opening [VNC client `http://localhost:12345`](http://localhost:12345) in your browser.

::: tip
Learn more about [VNC client](devcontainer#detailed).
:::
