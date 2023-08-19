all: deps build

.PHONY: deps build

deps:
	rosdep install --from-paths src -i -y

custom-deps:
	vcs import src --force --shallow < custom_deps.yaml

build:
	colcon build --symlink-install

sim:
	ROS_LOG_DIR=log/ ros2 launch src/openmower/launch/sim.launch.py

run:
	ROS_LOG_DIR=log/ ros2 launch src/openmower/launch/openmower.launch.py

rsp:
	ros2 launch src/openmower/launch/rsp.launch.py

remote-devices:
	sudo bash .devcontainer/scripts/remote_devices.sh 10.0.250.79 openmower