 
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
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
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
#include "warrior_interface/msg/lqr_debug_data.hpp"
#include "warrior_interface/msg/simulation_data.hpp"
#include "warrior_controller/warrior_handle.hpp"
// max leg length 0.41853056293485247
// min leg length 0.11914338936606465
//#define IMU_PLOT
//#define LK_PLOT
//#define GO1_PLOT
#define VMC_DEBUG
#define LQR_DEBUG
#define LEFT_CONTROLLER_INDEX 0
#define RIGHT_CONTROLLER_INDEX 1 
#define DRIVER_RADIUS 0.0875f
#define G01_REDUCTION_RATIO 6.33f
#define SIMULATION
//#define NO_SIMULATION

#ifdef NO_SIMULATION
/// left leg go1 param
#define GO1_0_ZEROS  0.7106166481971741f
#define GO1_3_ZEROS  0.0668541805587236f
/// right leg go1 param
#define GO1_1_ZEROS  4.24088191986084f
#define GO1_2_ZEROS  5.711969375610352f
/// leg common range
#define MAX_L0 0.23853056293485247f
#define MIN_L0 0.16914338936606465f
#define LEFT_LEG_FAI_ZERO 3.48716784548467f
#define RIGHT_LEG_FAI_ZERO 3.48716784548467f
#define BODY_Mg  5.3 * 9.82f
#else
/// left leg go1 param
#define GO1_0_ZEROS  -0.0866031f
#define GO1_3_ZEROS  -0.357487f
/// right leg go1 param
#define GO1_1_ZEROS  -0.0880985f
#define GO1_2_ZEROS  -0.357563f
/// leg common range
#define MAX_L0 0.23853056293485247f
#define MIN_L0 0.16914338936606465f
#define LEFT_LEG_FAI_ZERO 3.48716784548467f
#define RIGHT_LEG_FAI_ZERO 3.48716784548467f
#define BODY_Mg  5.3 * 9.82f
#endif
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

                double wx;
                double wy;
                double wz;

                double ax;
                double ay;
                double az;
                data_used_from_interface() {memset(this,0,sizeof(data_used_from_interface));}
            };
            struct send_data
            {
                double left_T1,left_T2;
                double right_T1,right_T2;
                double T_W;
                double left_tau_w;
                double right_tau_w;
                double left_Tp;
                send_data() {memset(this,0,sizeof(send_data));}
            };
            struct controller_feedback
            {
                float ch_l_x;
                float ch_l_y;
                float ch_r_x;
                float ch_r_y;
                uint8_t sw_l;
                uint8_t sw_r;
                float wheel;
                controller_feedback() {memset(this,0,sizeof(controller_feedback));}
            };
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
            /* balance controller */
            struct state_variables
            {
                double x;       //^
                double x_dot;   //^
                double theta_now;   
                double theta_last;
                double theta_dot;
                double fai;
                double fai_dot;
               state_variables()  : x(0),x_dot(0),theta_now(0),theta_last(0)
                                    ,theta_dot(0),fai(0),fai_dot(0){}
            };
            struct leg_balance_controller
            {
                MatrixXd K;
                MatrixXd X_d;
                MatrixXd X;
                MatrixXd U;
                leg_balance_controller() : K(2,6), X_d(1,6),X(1,6),U(2,1){}
            };
            /*lqr controller*/
            struct leg_lqr_param{
                MatrixXd A_;
                MatrixXd B_;
                MatrixXd Q_;
                MatrixXd R_;
                MatrixXd K_;
                MatrixXd P_;
                leg_lqr_param() : A_(6,6),B_(6,2),Q_(6, 6), R_(2, 2) , K_(2, 6), P_(2, 2){}
            };

            /*GO1*/
            std::shared_ptr<Go1Handle> Go1_LF_handles_;
            std::shared_ptr<Go1Handle> Go1_RF_handles_;
            std::shared_ptr<Go1Handle> Go1_RB_handles_;
            std::shared_ptr<Go1Handle> Go1_LB_handles_;
            std::vector<std::string> leg_joint_name_;
            /*MF9025*/
            std::shared_ptr<LK9025Handle> LK_L_handles_;
            std::shared_ptr<LK9025Handle> LK_R_handles_;
            std::vector<std::string> wheel_joint_name_;
            /*imu*/
            std::shared_ptr<ImuHandle> imu_handles_;
            std::vector<std::string> imu_joint_name_;
            send_data send_data_;
            data_used_from_interface need_data_form_hi_;
            rc_commmonds rc_commmonds_;
            controller_feedback real_feed_back_,simu_feedback_;
            state_variables left_destination_,left_set_feedback_,state_var_tar_;
            state_variables right_destination_,right_set_feedback_,state_var_now_;
            leg_balance_controller leg_balance_controller_[2];
            leg_balance_controller balance_controller_;
            double body_mg_;
            double pitch_now_,pitch_last_;//use and update in updateX
            leg_lqr_param left_leg_lqr_param_,right_leg_lqr_param_;
            leg_lqr_param leg_lqr_param_;
            std::shared_ptr<LQR> left_lqr_;
            std::shared_ptr<LQR> right_lqr_;
            std::shared_ptr<LQR> lqr_;
            std::shared_ptr<five_bar_linkage::FiveBar> left_five_bar_;
            std::shared_ptr<five_bar_linkage::FiveBar> right_five_bar_;
            std::shared_ptr<VMC> left_vmc_;
            std::shared_ptr<VMC> right_vmc_;
            std::shared_ptr<MiniPID> left_Fy_pid_;
            std::shared_ptr<MiniPID> right_Fy_pid_;
            /// remote data suscription.
            rclcpp::Subscription<warrior_interface::msg::DbusData>::SharedPtr command_subsciption_;
            /// remote subcription data buffer.
            realtime_tools::RealtimeBuffer<std::shared_ptr<warrior_interface::msg::DbusData>> command_ptr_;
            /// simulation data suscription.
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr simu_imu_subsciption_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr simu_leg_lb_p_subsciption_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr simu_leg_lb_v_subsciption_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr simu_leg_lf_p_subsciption_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr simu_leg_lf_v_subsciption_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr simu_leg_rb_p_subsciption_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr simu_leg_rb_v_subsciption_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr simu_leg_rf_p_subsciption_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr simu_leg_rf_v_subsciption_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr simu_wheelleft_p_subsciption_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr simu_wheelleft_v_subsciption_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr simu_wheelright_p_subsciption_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr simu_wheelright_v_subsciption_;
            /// simulation subcription data buffer.
            realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::Imu>>  simu_imu_ptr_;
            realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> simu_leg_lb_p_ptr_;
            realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> simu_leg_lb_v_ptr_;
            realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> simu_leg_lf_p_ptr_;
            realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> simu_leg_lf_v_ptr_;
            realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> simu_leg_rb_p_ptr_;
            realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> simu_leg_rb_v_ptr_;
            realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> simu_leg_rf_p_ptr_;
            realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> simu_leg_rf_v_ptr_;
            realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> simu_wheelleft_p_ptr_;
            realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> simu_wheelleft_v_ptr_;
            realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> simu_wheelright_p_ptr_;
            realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>> simu_wheelright_v_ptr_;
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
            //LQR controller data publisher
            std::shared_ptr<rclcpp::Publisher<warrior_interface::msg::LQRDebugData>> LQR_debug_data_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<warrior_interface::msg::LQRDebugData>>realtime_LQR_debug_data_publisher_ = nullptr;
            ////////////////////// Webots simulation data publisher ///////////////
            std::shared_ptr<rclcpp::Publisher<warrior_interface::msg::SimulationData>> Simulation_data_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<warrior_interface::msg::SimulationData>>realtime_Simulation_data_publisher_ = nullptr;
            std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> torque_lb_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>realtime_torque_lb_publisher_ = nullptr;
            std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> torque_lf_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>realtime_torque_lf_publisher_ = nullptr;
            std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> torque_rb_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>realtime_torque_rb_publisher_ = nullptr;
            std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> torque_rf_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>realtime_torque_rf_publisher_ = nullptr;
            std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> torque_wl_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>realtime_torque_wl_publisher_ = nullptr;
            std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> torque_wr_publisher_ = nullptr;
            std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>realtime_torque_wr_publisher_ = nullptr;

            std::shared_ptr<Go1Handle> get_Go1_handle(const std::string & joint_name);
            std::shared_ptr<LK9025Handle> get_LK_handle(const std::string & joint_name);
            std::shared_ptr<ImuHandle> get_angle(const std::string & joint_name);
            void initLQRParam(void);
            void setLegLQRGain(MatrixXd K,uint8_t index);
            void setLegLQRGain(MatrixXd K);
            void setLegLQRXd(uint8_t index);
            void setLegLQRXd(void);
            void setLegLQRX(uint8_t index);
            void setLegLQRX(void);
            void calclegLQRU(uint8_t index);
            void calclegLQRU(void);
            void updateXdes(uint8_t index);
            void updateXdes(void);
            void updateX(uint8_t index);
            void updateX(void);
            controller_interface::return_type updatingRemoteData(void);
            controller_interface::return_type updatingSimuImuData(void);
            controller_interface::return_type updatingSimuMotorData(void);
            /// update the data at first of time
            void updateDataFromInterface(void);
            /// get 9025 distance
            void update9025TotalDis(void);
            /// initialize the zero of lk9025 encoder.
            controller_interface::return_type init9025EncoderZeros(void);
    };

}

#endif // __WARRIOR_CONTROLLER__TEST_CONTROLLER_H__
