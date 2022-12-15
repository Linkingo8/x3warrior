#ifndef WARRIOR_HARDWARE_GO1_CONFIG_HPP_
#define WARRIOR_HARDWARE_GO1_CONFIG_HPP_
#include <string>
#include <cstring>
#include <vector>
#include "warrior_hardware/crc.hpp"
///////////////////////////////////////////////////////////////
//id must less than 3 instead leading to over memory problem///
///////////////////////////////////////////////////////////////
#define GO1_NUM 4
#define LF_GO1 0x00
#ifndef PI 
    #define PI 3.14159265
#endif
namespace warrior_hardware
{
    class Go1Config
    {
        public:
            Go1Config();
            ~Go1Config();
            int open(const std::string & port_name);
            int close();
            int read_frames(uint8_t* data, size_t size);
            int write_frame(const uint8_t* data, size_t size);
        private:
            int go1_port_;   
    };
    
    class Go1DataProcess : public crc
    {
        /**
        * @brief Motor mode control information
        * 
        */
        typedef struct
        {
            uint8_t id     :4;      // ID: 0,1...,14 15
            uint8_t status :3;      // work mode: 0.locked 1.FOC closed loop  2.encode correction  3-7.reserve
            uint8_t none   :1;      // reserved bits
        } RIS_Mode_t;// control mode 1Byte
        /**
        * @brief Motor state control states 
        * 
        */
        typedef struct
        {
            int16_t tor_set;        // Expect the joint to output torque unit: N.m     (q8)
            int16_t spd_set;        // Expect the joint to output speed  unit: rad/s   (q8)
            int32_t pos_set;        // Expect the joint to output position unit: rad     (q15)
            uint16_t  k_pos;        // Expect the joint Stiffness factor unit: 0.0-1.0 (q15)
            uint16_t  k_spd;        // Expect the joint damping factor unit: 0.0-1.0 (q15)
        } RIS_Comd_t;   // control parameters 12Byte
        /**
         * @brief 电机状态反馈信息
         * 
         */
        typedef struct
        {
            int16_t  torque;        // 实际关节输出扭矩 unit: N.m     (q8)
            int16_t  speed;         // 实际关节输出速度 unit: rad/s   (q8)
            int32_t  pos;           // 实际关节输出位置 unit: W       (q15)
            int8_t   temp;          // 电机温度: -128~127°C 90°C时触发温度保护
            uint8_t  MError :3;     // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
            uint16_t force  :12;    // 足端气压传感器数据 12bit (0-4095)
            uint8_t  none   :1;     // 保留位
        } RIS_Fbk_t;   // 状态数据 11Byte
        /**
         * @brief Motor data feedback
         * 
         */
        typedef struct
        {
            uint8_t head[2];    // 包头         2Byte
            RIS_Mode_t mode;    // 电机控制模式  1Byte
            RIS_Fbk_t   fbk;    // 电机反馈数据 11Byte
            uint16_t  CRC16;    // CRC          2Byte
        } MotorData_t;      // 电机返回数据     16Byte
        typedef struct
        {
            double velocity;
            double position;    
            double torque;
        } MotorData_Export_t;      // 电机返回数据     16Byte
        /**
        * @brief 控制数据包格式
        * 
        */
        typedef struct
        {
            union{  
                uint8_t tx_buff[17];
                struct{
                    uint8_t head[2];    // head                 2Byte
                    RIS_Mode_t mode;    // motor control mode   1Byte
                    RIS_Comd_t comd;    // motor expected date  12Byte
                    uint16_t   CRC16;   // CRC                  2Byte
                }data;
            }tx;
        } ControlData_t;    // host control commands     17Byte

        public:
            Go1DataProcess(uint16_t CRC16_CCITT_INIT);/*crc param init*/
            /* recieve */
            void Go1_data_rec(uint8_t id,uint8_t *buff_temp);
            double Go1_velocities_export(uint8_t id_temp);
            double Go1_positions_export(uint8_t id_temp);
            double Go1_torques_export(uint8_t id_temp);
            /* send */
            void Go1_head_set(void);
            void Go1_id_set(void);
            void Go1_speed_set(uint8_t index,double k_sped,double spd_set);
            void Go1_speed_set(uint8_t index,double spd_set);//damping mode
            void Go1_torque_set(uint8_t index,double tor_set);
            void Go1_zero_torque_set(uint8_t index);
            void Go1_position_set(uint8_t index,double k_pos,double pos_set);
            void Go1_crc_append(void);
            uint8_t* Go1_buff_get(uint8_t index);
            /* debug */
            void Go1_head_print(void);
            void Go1_buff_print(void);
        private:
            MotorData_t go1_feedback_data_[4];
            ControlData_t go1_control_data_[4];
            MotorData_Export_t go1_export_data_[4];
    };
}
#endif