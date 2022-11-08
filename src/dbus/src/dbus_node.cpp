#include "dbus/dbus_node.h"


void dbus_node::timer_callback()
{
    if(dbus_.get_uart_state())
    {
        dbus_.read();
        dbus_.getData(&rc_);
    }

    dbus_pub_->publish(rc_);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dbus_node>("dbus");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


