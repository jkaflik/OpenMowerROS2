REMOTE_HOST ?= omdev.local
REMOTE_USER ?= openmower
ROS_LOG_DIR = log/

all: custom-deps deps build

.PHONY: deps build

dev-containers:
	cd .devcontainer && docker-compose up -d

# turtlebot3_gazebo does not have a build on iron arm64
deps:
	rosdep install --from-paths ./ -i -y -r

custom-deps:
	sh utils/install-custom-deps.sh

build-libs:
	colcon build --base-paths "src/lib/*"

build:
	colcon build --symlink-install

sim:
	killall -9 ruby || true
	ros2 launch launch/sim.launch.py

run:
	ros2 launch launch/openmower.launch.py

dev:
	cd .devcontainer && docker-compose up -d

run-realsense:
	ros2 launch realsense2_camera rs_launch.py initial_reset:=true pointcloud.enable:=true publish_tf:=false enable_infra1:=true enable_depth:=true enable_gyro:=true enable_accel:=true unite_imu_method:=1

run-foxglove:
	ros2 launch foxglove_bridge foxglove_bridge_launch.xml

rsp:
	ros2 launch launch/rsp.launch.py

remote-devices:
	sudo bash .devcontainer/scripts/remote_devices.sh $(REMOTE_HOST) $(REMOTE_USER)
