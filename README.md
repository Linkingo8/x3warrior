# x3warrior
warrior ros2 project for x3pi 

# visualization

## use gui in wsl2
export DISPLAY=<wsl ipv4>:0
source /etc/profile

# run
source install/setup.bash
ros2 launch waarior_bringup warrior.py

# ssh x3
ssh sunrise@192.168.189.253
scp -r ../x3warrior sunrise@192.168.189.253:/home/sunrise
# debug
    RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go1_control_data_[position].tx.data.comd.k_spd %x",go1_control_data_[index].tx.data.comd.k_spd);    
