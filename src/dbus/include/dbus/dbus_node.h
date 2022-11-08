#ifndef DBUS_NODE_H_
#define DBUS_NODE_H_
#include "dbus.h"
#include "rclcpp/rclcpp.hpp"
#include "interface/msg/dbus_data.hpp"


class dbus_node : public rclcpp::Node
{

public:
    dbus_node(std::string name) : Node(name)
    {
        dbus_pub_ = this->create_publisher<interface::msg::DbusData>("/rc_msg", 1);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(16), std::bind(&dbus_node::timer_callback, this));
        dbus_.init("/dev/ttyS3");
    }
        
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interface::msg::DbusData>::SharedPtr dbus_pub_;
    void timer_callback();
    DBus dbus_{};
    interface::msg::DbusData rc_;
};


#endif  