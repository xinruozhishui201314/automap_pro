# AutoMap-Pro Build Convenience Makefile (ROS2 Humble)
SHELL := /bin/bash
WS    := $(HOME)/automap_ws
PKG   := automap_pro

.PHONY: setup build build-release test run-online run-offline clean docker-build docker-run

setup:
	@echo "=== Setting up AutoMap-Pro workspace (ROS2 Humble) ==="
	mkdir -p $(WS)/src
	ln -sfn $(CURDIR) $(WS)/src/$(PKG) || true
	cd $(WS) && source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -r -y || true

build:
	@echo "=== Building (Debug) ==="
	cd $(WS) && source /opt/ros/humble/setup.bash && colcon build --packages-select $(PKG) --cmake-args -DCMAKE_BUILD_TYPE=Debug

build-release:
	@echo "=== Building (Release) ==="
	cd $(WS) && source /opt/ros/humble/setup.bash && colcon build --packages-select $(PKG) --cmake-args -DCMAKE_BUILD_TYPE=Release

test:
	@echo "=== Running tests ==="
	cd $(WS) && source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select $(PKG)

run-online:
	@echo "=== Starting AutoMap-Pro (Online mode, ROS2) ==="
	cd $(WS) && source /opt/ros/humble/setup.bash && source install/setup.bash && \
	  ros2 launch $(PKG) automap_online.launch.py config:=$(CURDIR)/config/system_config.yaml

run-offline: BAG_FILE ?= $(HOME)/data/mapping
run-offline:
	@echo "=== Starting AutoMap-Pro (Offline mode, bag=$(BAG_FILE)) ==="
	cd $(WS) && source /opt/ros/humble/setup.bash && source install/setup.bash && \
	  ros2 launch $(PKG) automap_offline.launch.py \
	    config:=$(CURDIR)/config/system_config.yaml \
	    bag_file:=$(BAG_FILE)

save-map: OUTPUT_DIR ?= /data/automap_output
save-map:
	ros2 service call /automap/save_map automap_pro/srv/SaveMap \
	  "{output_dir: '$(OUTPUT_DIR)', save_pcd: true, save_ply: true, save_las: false, save_trajectory: true}"

trigger-opt:
	ros2 service call /automap/trigger_optimize automap_pro/srv/TriggerOptimize "{full_optimization: true, max_iterations: 100}"

status:
	ros2 service call /automap/get_status automap_pro/srv/GetStatus "{}"

clean:
	cd $(WS) && rm -rf build/$(PKG) install/$(PKG) log/$(PKG) || true

docker-build:
	docker build -t automap_pro:latest -f docker/Dockerfile .

docker-run:
	docker-compose -f docker/docker-compose.yml up automap

eval-traj: EST ?= /data/automap_output/trajectory/optimized_trajectory_tum.txt
eval-traj: REF ?= /data/groundtruth.txt
eval-traj:
	python3 scripts/evaluate_trajectory.py --est $(EST) --ref $(REF)

eval-map: MAP ?= /data/automap_output/map/global_map.pcd
eval-map:
	python3 scripts/evaluate_map.py --map $(MAP)

visualize: DIR ?= /data/automap_output
visualize:
	python3 scripts/visualize_results.py --output_dir $(DIR)
