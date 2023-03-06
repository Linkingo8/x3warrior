#ifndef __WARRIOR_CONTROLLER__WARRIOR_HANDLE_H__
#define __WARRIOR_CONTROLLER__WARRIOR_HANDLE_H__
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
namespace warrior_controller
{
    class ImuHandle
    {
        public:
            ImuHandle(
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> pitch,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> yaw,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> roll,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> wx,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> wy,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> wz
                );

            double get_pitch(){ return pitch_.get().get_value(); };
            double get_yaw(){ return yaw_.get().get_value(); };
            double get_roll(){ return roll_.get().get_value(); };

            double get_wx(){ return wx_.get().get_value(); };
            double get_wy(){ return wy_.get().get_value(); };
            double get_wz(){ return wz_.get().get_value(); };

        private:
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> pitch_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> yaw_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> roll_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> wx_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> wy_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> wz_;

    };


    class LK9025Handle
    {
        public:
            LK9025Handle(
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> acceleration,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_ref,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_ref,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> torque_ref
            );
            
            void set_position(double value){ position_ref_.get().set_value(value); };
            void set_velocity(double value){ velocity_ref_.get().set_value(value); };
            void set_torque(double value){ torque_ref_.get().set_value(value); };

            double get_position(){ return position_.get().get_value(); };
            double get_velocity(){ return velocity_.get().get_value(); };
            double get_acceleration(){ return acceleration_.get().get_value(); };

        private:
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> acceleration_;
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_ref_;
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_ref_;
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> torque_ref_;
    };
    class Go1Handle
    {
        typedef struct
        {
            uint8_t id     :4;      // 电机ID: 0,1...,14 15表示向所有电机广播数据(此时无返回)
            uint8_t status :3;      // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
            uint8_t none   :1;      // 保留位
        } RIS_Mode_t;   // 控制模式 1Byte
        typedef struct
        {
            int16_t tor_des;        // 期望关节输出扭矩 unit: N.m     (q8)
            int16_t spd_des;        // 期望关节输出速度 unit: rad/s   (q8)
            int32_t pos_des;        // 期望关节输出位置 unit: rad     (q15)
            uint16_t  k_pos;        // 期望关节刚度系数 unit: 0.0-1.0 (q15)
            uint16_t  k_spd;        // 期望关节阻尼系数 unit: 0.0-1.0 (q15)
        } RIS_Comd_t;   // 控制参数 12Byte
        typedef struct
        {
            uint8_t head[2];    // 包头         2Byte
            RIS_Mode_t mode;    // 电机控制模式  1Byte
            RIS_Comd_t comd;    // 电机期望数据 12Byte
            uint16_t   CRC16;   // CRC          2Byte
        } ControlData_t;    // 主机控制命令     17Byte
        struct MotorCmd{
        // 定义 发送格式化数据
        public:
            int hex_len = 17;                   
            unsigned short id;              // 电机ID 0~14 15:广播ID 此时电机无返回
            unsigned short mode;            // 电机模式 0:刹车 1:FOC闭环 2:电机标定
            float T;                        // 期望关节的输出力矩(电机转子转矩 N.m) 范围: ±127.99
            float W;                        // 期望关节速度(电机转子转速 rad/s)         ±804.00
            float Pos;                      // 期望关节位置(电机转子位置 rad)           ±411774
            float K_P;                      // 关节刚度系数                           0~25.599
            float K_W;                      // 关节速度系数                           0~25.599
            ControlData_t  motor_send_data;  //电机控制数据结构体，详见motor_msg.h
        };
        
        public:
            Go1Handle(
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> acceleration,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_ref,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_ref,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> torque_ref,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> damp_ref,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> zero_torque_ref,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> torque_and_position_ref
            );
            
            void set_position(double value){ position_ref_.get().set_value(value); };
            void set_velocity(double value){ velocity_ref_.get().set_value(value); };
            void set_torque(double value){ torque_ref_.get().set_value(value); };
            void set_damp(double value){ damp_ref_.get().set_value(value); };
            void set_zero_torque(double value){ zero_torque_ref_.get().set_value(value); };
            void set_position_and_motor(double value){ torque_and_position_ref_.get().set_value(value); };

            double get_position(){ return position_.get().get_value(); };
            double get_velocity(){ return velocity_.get().get_value(); };
            double get_acceleration(){ return acceleration_.get().get_value(); };

        private:
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_;
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_;
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> acceleration_;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_ref_;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_ref_;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> torque_ref_;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> damp_ref_;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> zero_torque_ref_;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> torque_and_position_ref_;
    };
}
#endif // __WARRIOR_CONTROLLER__TEST_HANDLE_H__