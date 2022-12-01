
#ifndef __WARRIOR_CONTROLLER__WHEEL_BALANCING_CONTROLLER_H__
#define __WARRIOR_CONTROLLER__WHEEL_BALANCING_CONTROLLER_H__

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <string>

#include "warrior_controller/warrior_controller_compiler.h"
#include "warrior_controller/imu_handle.hpp"

namespace warrior_controller
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    class WheelBalancingController
        : public controller_interface::ControllerInterface
    {
        public:
            WARRIOR_CONTROLLER_PUBLIC
            WheelBalancingController();
            
            WARRIOR_CONTROLLER_PUBLIC
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            WARRIOR_CONTROLLER_PUBLIC
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            WARRIOR_CONTROLLER_PUBLIC
            controller_interface::return_type init(const std::string & controller_name) override;

            WARRIOR_CONTROLLER_PUBLIC
            controller_interface::return_type update() override;

            WARRIOR_CONTROLLER_PUBLIC
            CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            WARRIOR_CONTROLLER_PUBLIC
            CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            WARRIOR_CONTROLLER_PUBLIC
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            WARRIOR_CONTROLLER_PUBLIC
            CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

            WARRIOR_CONTROLLER_PUBLIC
            CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

            WARRIOR_CONTROLLER_PUBLIC
            CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        


        private:

            std::shared_ptr<ImuHandle> imu_handles_;
            std::shared_ptr<ImuHandle> get_angle(const std::string & joint_name);
            std::vector<std::string> imu_joint_name_;

    };
}

#endif // __WARRIOR_CONTROLLER__TEST_CONTROLLER_H__
