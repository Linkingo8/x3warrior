#ifndef WARRIOR_HARDWARE_BSP_LK_MF9025_HPP_
#define WARRIOR_HARDWARE_BSP_LK_MF9025_HPP_
#include "warrior_hardware/can_driver.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <string>
#include <cstring>
#include <vector>
///////////////////////////////////////////////////////////////
//id must less than 2 instead leading to over memory problem///
///////////////////////////////////////////////////////////////
#define GO1_NUM 4

#define LEFT_ID  0x141 //0
#define RIGHT_ID 0x142 //1

#ifndef PI 
    #define PI 3.14159265
#endif
namespace warrior_hardware
{
    typedef struct LK_MF9025_rec
    {
        /* data */
        uint8_t commond;
        int8_t temperature;
        int16_t iq;
        int16_t speed;
        uint16_t encoder;
    }LK_MF9025_rec;
    
    class MF9025DataProcess
    {
        public:
            /*int32_t 0.01dps/LSB*/
            void MF9025_speed_set(uint8_t id_shift, double speed);
            void MF9025_torque_set(uint8_t id_shift, double speed);
            void MF9025_position_set(uint8_t id_shift, double speed);
            void MF9025_commond_send(uint16_t id);
            void MF9025_message_rec(VCI_CAN_OBJ *rec_,int16_t q1);
        private:
        /*0: left 1: right*/
            VCI_CAN_OBJ send_9025_[2];
            LK_MF9025_rec rec_9025_[2];
    };
}
#endif