.PHONY: all build clean deploy restart

all: build

build:
	bash -c "source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -r -y && colcon build"
	@echo "Run \"source ./install/setup.bash\""

deploy:
	bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select robot_interfaces robot_description robot_webrtc robot_kinematics robot_bringup motor_handler"
	systemctl is-enabled --quiet robot-teleop || sudo systemctl enable robot-teleop
	sudo systemctl restart robot-teleop
	@echo "Deployed and restarted robot-teleop"

restart:
	sudo systemctl restart robot-teleop

clean:
	rm -rf build install log
