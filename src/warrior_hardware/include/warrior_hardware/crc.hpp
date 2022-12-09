#ifndef WARRIOR_HARDWARE_CRC_HPP_
#define WARRIOR_HARDWARE_CRC_HPP_
#include <cstdint>
#include <cstddef>
/**
@improvement 
    1. the init value of crc could be added into constructor;
    2. crc table
*/
namespace warrior_hardware
{
    class crc{
        public:
            crc();
            ~crc();
            uint16_t CRC16_CCITT(uint8_t* bytes,int dwLength);
            void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
        private:

    };

}

#endif