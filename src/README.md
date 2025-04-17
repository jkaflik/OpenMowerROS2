# `/src` structure

All nodes are expected to be implemented in C++ and are organized into subdirectories based on their functionality. The directory structure is designed to facilitate easy navigation and understanding of the project.

## Directory Structure

- `action/` - Action type definitions (e.g., Dock.action)
- `behavior_trees/` - Behavior tree XML definitions for robot behaviors
- `lib/` - Library code from external dependencies
- `msg/` - ROS message type definitions for project-specific message types
- `sim/` - Simulation components and code
- `srv/` - ROS service type definitions
- `**/` - Nodes and components for the project, organized by functionality

## Contribution Guidelines

When adding new code to this project:

1. Place new ROS message definitions in the `msg/` directory
2. Place new ROS service definitions in the `srv/` directory
3. Place new ROS action definitions in the `action/` directory
4. Nodes and components should be placed in the appropriate subdirectory under `src/` based on their functionality
   - For example, if you are adding a new node for docking action server, place it in `src/docking_action_server`
   - Do not organize `.cpp` and `.hpp` files into separate directories. Keep header and source files together in the same directory.
5. Organize code by functionality in appropriately named subdirectories
6. Use shared `/CMakeLists.txt`, add your node as a target in the `CMakeLists.txt` file.
7. Use shared `/package.xml`, add your node as a dependency in the `package.xml` file. 
8. Follow the existing code style and conventions used in the project

## Building

The code in this directory is built using the colcon build system. Run `make build` from the workspace root directory to build all components.