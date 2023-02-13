 
#ifndef __WARRIOR_CONTROLLER__WHEEL_BALANCING_CONTROLLER_H__
#define __WARRIOR_CONTROLLER__WHEEL_BALANCING_CONTROLLER_H__

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <string>
#include <Eigen/Dense>
#include <iostream>
#include "std_msgs/msg/string.hpp"
#include "warrior_controller/warrior_controller_compiler.h"
#include "warrior_common/lqr.hpp"
#include "warrior_interface/msg/dbus_data.hpp"
#include "warrior_interface/msg/imu_data.hpp"
#include "warrior_interface/msg/lk9025_feedback.hpp"
#include "warrior_interface/msg/go1_feedback.hpp"
#include "warrior_controller/warrior_handle.hpp"

#define IMU_PLOT
#define LK_PLOT
#define GO1_PLOT
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
            /*GO1*/
            std::shared_ptr<Go1Handle> Go1_LF_handles_;
            std::shared_ptr<Go1Handle> Go1_RF_handles_;
            std::shared_ptr<Go1Handle> Go1_LB_handles_;
            std::shared_ptr<Go1Handle> Go1_RB_handles_;
            std::shared_ptr<Go1Handle> get_Go1_handle(const std::string & joint_name);
            std::vector<std::string> leg_joint_name_;
            /*MF9025*/
            std::shared_ptr<LK9025Handle> LK_L_handles_;
            std::shared_ptr<LK9025Handle> LK_R_handles_;
            std::shared_ptr<LK9025Handle> get_LK_handle(const std::string & joint_name);
            std::vector<std::string> wheel_joint_name_;
            /*imu*/
            std::shared_ptr<ImuHandle> imu_handles_;
            std::shared_ptr<ImuHandle> get_angle(const std::string & joint_name);
            std::vector<std::string> imu_joint_name_;
            controller_interface::return_type updatingRemoteData(void);
            /*remote*/
            struct rc_commmonds
            {
                float ch_l_x;
                float ch_l_y;
                float ch_r_x;
                float ch_r_y;
                uint8_t sw_l;
                uint8_t sw_r;
                float wheel;

                rc_commmonds() : ch_l_x(0.0), ch_l_y(0.0), ch_r_x(0.0)
                                 ,ch_r_y(0.0), sw_l(1),sw_r(1),wheel(0) {}
            };
            /*lqr controller*/
            MatrixXd A_;
            MatrixXd B_;
            MatrixXd Q_;
            MatrixXd R_;
            MatrixXd K_;
            MatrixXd P_;
            std::shared_ptr<LQR> lqr_;
            void initLQRParam(void);
            rc_commmonds rc_commmonds_;
            /// remote data suscription.
            rclcpp::Subscription<warrior_interface::msg::DbusData>::SharedPtr command_subsciption_;
            /// remote subcription data buffer.
            realtime_tools::RealtimeBuffer<std::shared_ptr<warrior_interface::msg::DbusData>> command_ptr_;
            ///imu data publisher 
            std::shared_ptr<rclcpp::Publisher<warrior_interface::msg::ImuData>> imu_data_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<warrior_interface::msg::ImuData>>realtime_imu_data_publisher_ = nullptr;
            ///9025 data publisher 
            std::shared_ptr<rclcpp::Publisher<warrior_interface::msg::LK9025Feedback>> LK9025_data_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<warrior_interface::msg::LK9025Feedback>>realtime_LK9025_data_publisher_ = nullptr;
            ///Go1 data publisher 
            std::shared_ptr<rclcpp::Publisher<warrior_interface::msg::Go1Feedback>> Go1_data_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<warrior_interface::msg::Go1Feedback>>realtime_Go1_data_publisher_ = nullptr;
    };
}

#endif // __WARRIOR_CONTROLLER__TEST_CONTROLLER_H__
