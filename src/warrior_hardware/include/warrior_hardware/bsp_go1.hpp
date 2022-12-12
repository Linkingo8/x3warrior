#ifndef WARRIOR_HARDWARE_GO1_CONFIG_HPP_
#define WARRIOR_HARDWARE_GO1_CONFIG_HPP_
#include <string>
#include <cstring>
#include <vector>
#include "warrior_hardware/crc.hpp"
#define GO1_NUM 4
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
            void Go1_head_set(void);
            void Go1_id_set(void);
            void Go1_speed_set(uint8_t index,double k_sped,double spd_set);
            void Go1_torque_set(uint8_t index,double tor_set);
            void Go1_position_set(uint8_t index,double k_pos,double pos_set);
            void Go1_crc_append(void);
            uint8_t* Go1_buff_get(uint8_t index);
            //debug
            void Go1_head_print(void);
            void Go1_buff_print(void);
        private:
            ControlData_t go1_control_data_[4];
            
    };
}
#endif