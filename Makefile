OPENMOWER_REMOTE_IP ?= 10.0.251.104
OPENMOWER_REMOTE_USER ?= openmower
ROS_LOG_DIR = log/

all: deps build

.PHONY: deps build

dev-containers:
	cd .devcontainer && docker-compose up -d

# turtlebot3_gazebo does not have a build on iron arm64
deps:
	rosdep install --from-paths src -i -y --skip-keys="turtlebot3_gazebo"

custom-deps:
	sh utils/install-custom-deps.sh

build:
	colcon build --symlink-install

sim:
	ros2 launch src/openmower/launch/sim.launch.py

run:
	ros2 launch src/openmower/launch/openmower.launch.py

run-realsense:
	ros2 launch realsense2_camera rs_launch.py initial_reset:=true pointcloud.enable:=true publish_tf:=false enable_infra1:=true enable_depth:=true enable_gyro:=true enable_accel:=true unite_imu_method:=1

run-foxglove:
	ros2 launch foxglove_bridge foxglove_bridge_launch.xml

rsp:
	ros2 launch src/openmower/launch/rsp.launch.py

remote-devices:
	sudo bash .devcontainer/scripts/remote_devices.sh $(OPENMOWER_REMOTE_IP) $(OPENMOWER_REMOTE_USER)
