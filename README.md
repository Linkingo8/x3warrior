# x3warrior
warrior ros2 project for x3pi 

# visualization
## use gui in wsl2 on win10
export DISPLAY=<wsl ipv4>:0
source /etc/profile
## use gui in wsl2 on win11
### bridge connection
#### powershell
Set-VMSwitch WSL -NetAdapterName WLAN 
#### wsl2
sudo ip addr del $(ip addr show eth0 | grep 'inet\b' | awk '{print $2}' | head -n 1) dev eth0
sudo ip addr add <ip_new>/24 broadcast <gateway.255> dev eth0  
**sudo ip addr add 192.168.213.10/24 broadcast 192.168.213.255 dev eth0**
sudo ip route add 0.0.0.0/0 via <gateway> dev eth0 
**sudo ip route add 0.0.0.0/0 via 192.168.213.162 dev eth0**
sudo vim /etc/resolv.conf  
nameserver <gateway>
**nameserver 192.168.213.162**

# compile
vim ./src/warrior_controller/src/wheel_balancing_controller.cpp
vim ./src/warrior_controller/include/warrior_controller/wheel_balanci·ng_controller.hpp 
vim ./src/warrior_hardware/src/go1_hardware_interface.cpp
vim ./src/warrior_description/config/robot_controller_config.yaml
vim +36 ./src/warrior_hardware/CMakeLists.txt
colcon build --symlink-install --parallel-workers 1

# run
colcon build --symlink-install
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch warrior_bringup warrior.py

# ssh x3
ssh sunrise@192.168.67.253
scp -r ../x3warrior sunrise@192.168.67.253:/home/sunrise
scp -r ./src/warrior_controller/src/wheel_balancing_controller.cpp  sunrise@192.168.67.253:~/x3warrior/src/warrior_controller/src
# debug
RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go1_control_data_[position].tx.data.comd.k_spd %x",go1_control_data_[index].tx.data.comd.k_spd);    
ros2 topic echo /vmc_debug_feedback

# simulation
roscore
source /opt/ros/noetic/setup.bash
ros2 run ros1_bridge dynamic_bridge
在仿真中要将go1的反馈和发送的减速比去 \
仿真中没有电机的编码器溢出，直接处理。\
