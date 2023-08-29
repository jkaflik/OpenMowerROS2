OPENMOWER_REMOTE_IP ?= 10.0.251.104
OPENMOWER_REMOTE_USER ?= openmower
ROS_LOG_DIR = log/

all: deps build

.PHONY: deps build

dev-containers:
	cd .devcontainer && docker-compose up -d

deps:
	rosdep install --from-paths src -i -y

custom-deps:
	sh utils/install-custom-deps.sh

build:
	colcon build --symlink-install

sim:
	ros2 launch src/openmower/launch/sim.launch.py

run:
	ros2 launch src/openmower/launch/openmower.launch.py

run-localization:
	ros2 launch src/openmower/launch/localization.launch.py

rsp:
	ros2 launch src/openmower/launch/rsp.launch.py

remote-devices:
	sudo bash .devcontainer/scripts/remote_devices.sh $(OPENMOWER_REMOTE_IP) $(OPENMOWER_REMOTE_USER)
