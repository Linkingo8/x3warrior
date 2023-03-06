# x3warrior
warrior ros2 project for x3pi 

# visualization

## use gui in wsl2
export DISPLAY=<wsl ipv4>:0
source /etc/profile
# compile
vim ./src/warrior_controller/src/wheel_balancing_controller.cpp
vim ./src/warrior_controller/include/warrior_controller/wheel_balancing_controller.hpp
vim ./src/warrior_description/config/robot_controller_config.yaml
vim ./src/warrior_hardware/src/go1_hardware_interface.cpp
vim ./src/warrior_description/config/robot_controller_config.yaml
colcon build --symlink-install --parallel-workers 1
# run
source /opt/ros/foxy/setup.bash
source install/setup.bash \
ros2 launch warrior_bringup warrior.py

ros2 topic echo /vm
# ssh x3
ssh sunrise@192.168.189.253
scp -r ../x3warrior sunrise@192.168.189.253:/home/sunrise
# debug
    RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go1_control_data_[position].tx.data.comd.k_spd %x",go1_control_data_[index].tx.data.comd.k_spd);    
