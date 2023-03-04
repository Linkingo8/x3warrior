 
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
#include <cmath>
#include "std_msgs/msg/string.hpp"
#include "warrior_controller/warrior_controller_compiler.h"
///common
#include "warrior_common/lqr.hpp"
#include "warrior_common/vmc.hpp"
#include "warrior_common/pid.hpp"
#include "warrior_common/five_bar_linkage.hpp"
///hardware
#include "warrior_interface/msg/dbus_data.hpp"
#include "warrior_interface/msg/imu_data.hpp"
#include "warrior_interface/msg/lk9025_feedback.hpp"
#include "warrior_interface/msg/go1_feedback.hpp"
#include "warrior_interface/msg/vmc_debug_data.hpp"
#include "warrior_controller/warrior_handle.hpp"

//#define IMU_PLOT
//#define LK_PLOT
#define GO1_PLOT
#define VMC_DEBUG
#define LEFT_CONTROLLER_INDEX 0
#define RIGHT_CONTROLLER_INDEX 1
#define DRIVER_RADIUS 0.0775f
#define G01_REDUCTION_RATIO 6.33f
/// left leg go1 param
#define GO1_0_ZEROS  0.0874224f
#define GO1_3_ZEROS -0.812973f
/// right leg go1 param
#define GO1_1_ZEROS  0.0874224f
#define GO1_2_ZEROS -0.812973f

#define LEFT_LEG_FAI_ZERO 1.5662f
#define RIGHT_LEG_FAI_ZERO 1.5662f

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
            std::shared_ptr<Go1Handle> Go1_RB_handles_;
            std::shared_ptr<Go1Handle> Go1_LB_handles_;
            
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
            /*get data from interface*/
            struct data_used_from_interface
            {
                double left_lk9025_pos;
                double left_lk9025_vel;
                double left_lk9025_tor;
                double left_lk9025_ecoder_zero;
                uint16_t left_lk9025_ecoder_last;
                int32_t left_lk9025_circle_cnt;
                uint8_t left_init_flag;

                double right_lk9025_pos;
                double right_lk9025_vel;
                double right_lk9025_tor;
                double right_lk9025_ecoder_zero;
                uint16_t right_lk9025_ecoder_last;
                int32_t right_lk9025_circle_cnt;
                uint8_t right_init_flag;

                double right_leg_total_dis;
                double right_leg_dis_dot;
                double left_leg_total_dis;
                double left_leg_dis_dot;
                double left_leg_fai1;
                double left_leg_fai4;
                double right_leg_fai1;
                double right_leg_fai4;

                double lf_go1_pos;
                double lf_go1_vel;
                double lf_go1_tor;
                double lf_go1_zero_fai;

                double rf_go1_pos;
                double rf_go1_vel;
                double rf_go1_tor;

                double left_fai1_;
                double left_fai4_;

                double rb_go1_pos;
                double rb_go1_vel;
                double rb_go1_tor;
                
                double right_fai1_;
                double right_fai4_;

                double lb_go1_pos;
                double lb_go1_vel;
                double lb_go1_tor;

                double pitch;
                double yaw;
                double roll;
                data_used_from_interface() {memset(this,0,sizeof(data_used_from_interface));}
            };
            struct send_data
            {
                double left_T1,left_T2;
                double right_T1,right_T2;
                double T_W;
                send_data() {memset(this,0,sizeof(send_data));}
            };
            send_data send_data_;
            data_used_from_interface need_data_form_hi_;
            /// update the data at first of time
            void updateDataFromInterface(void);
            /// get 9025 distance
            void update9025TotalDis(void);
            /// initialize the zero of lk9025 encoder.
            void init9025EncoderZeros(void);
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
            rc_commmonds rc_commmonds_;
            /* balance controller */
            struct state_variables
            {
                double x;       //^
                double x_dot;   //^
                double theta;   
                double theta_dot;
                double fai;
                double fai_dot;
               state_variables()  : x(0),x_dot(0),theta(0)
                                    ,theta_dot(0),fai(0),fai_dot(0){}
            };
            state_variables left_destination_,left_set_feedback_;

            struct leg_balance_controller
            {
                MatrixXd K;
                MatrixXd X_d;
                MatrixXd X;
                MatrixXd U;
                leg_balance_controller() : K(2,6), X_d(1,6),X(1,6),U(2,1){}
            };
            leg_balance_controller leg_balance_controller_[2];
            double pitch_now_,pitch_last_;//use and update in updateX
            void setLegLQRGain(MatrixXd K,uint8_t index);
            void setLegLQRXd(uint8_t index);
            void setLegLQRX(uint8_t index);
            void calclegLQRU(uint8_t index);
            void updateXdes(uint8_t index);
            void updateX(uint8_t index);
            // void InitXdes(uint8_t index);
            /*lqr controller*/
            MatrixXd A_;
            MatrixXd B_;
            MatrixXd Q_;
            MatrixXd R_;
            MatrixXd K_;
            MatrixXd P_;
            std::shared_ptr<LQR> left_lqr_;
            std::shared_ptr<five_bar_linkage::FiveBar> left_five_bar_;
            std::shared_ptr<five_bar_linkage::FiveBar> right_five_bar_;
            std::shared_ptr<VMC> left_vmc_;
            std::shared_ptr<VMC> right_vmc_;
            std::shared_ptr<MiniPID> left_Fy_pid_;
            std::shared_ptr<MiniPID> right_Fy_pid_;
            void initLQRParam(void);
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
            ///VMC controller data publisher 
            std::shared_ptr<rclcpp::Publisher<warrior_interface::msg::VMCDebugData>> VMC_debug_data_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<warrior_interface::msg::VMCDebugData>>realtime_VMC_debug_data_publisher_ = nullptr;
    };
}

#endif // __WARRIOR_CONTROLLER__TEST_CONTROLLER_H__
