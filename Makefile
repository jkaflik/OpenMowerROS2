all: deps build

.PHONY: deps build

deps:
	rosdep install --from-paths src -i -y

custom-deps:
	vcs import src --force --shallow < custom_deps.yaml

build:
	colcon build --symlink-install

sim:
	ros2 launch src/openmower/launch/run.py
