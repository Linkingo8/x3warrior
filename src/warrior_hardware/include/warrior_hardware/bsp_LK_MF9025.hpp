#ifndef WARRIOR_HARDWARE_BSP_LK_MF9025_HPP_
#define WARRIOR_HARDWARE_BSP_LK_MF9025_HPP_
#include "warrior_hardware/can_driver.hpp"
#include <string>
#include <cstring>
#include <vector>
///////////////////////////////////////////////////////////////
//id must less than 2 instead leading to over memory problem///
///////////////////////////////////////////////////////////////
#define GO1_NUM 4

#define LEFT_ID  0x141
#define RIGHT_ID 0x142

#ifndef PI 
    #define PI 3.14159265
#endif
namespace warrior_hardware
{
    class MF9025DataProcess
    {
        public:
            /*int32_t 0.01dps/LSB*/
            void MF9025_speed_set(uint8_t id_shift, double speed);
            void MF9025_torque_set(uint8_t id_shift, double speed);
            void MF9025_position_set(uint8_t id_shift, double speed);
            void MF9025_commond_send(uint16_t id);
        private:
        /*0: left 1: right*/
            VCI_CAN_OBJ send_9025_[2];
    };
}
#endif