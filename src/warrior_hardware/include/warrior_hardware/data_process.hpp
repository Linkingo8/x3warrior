#ifndef WARRIOR_HARDWARE_DATA_PROCESS_HPP_
#define WARRIOR_HARDWARE_DATA_PROCESS_HPP_
#include <cstdint>
#include <cstddef>
#include <cmath>
/**
@improvement 
    1. the init value of crc could be added into constructor;
    2. crc table
*/
namespace warrior_hardware
{
    class BasicDataProcess{
        public:
            BasicDataProcess() = default;
            ~BasicDataProcess() = default;
            uint16_t charToHex(char *data, int length)
            {
                uint8_t buffer[100]{0};
                uint16_t result = 0;
                
                for(int i = 0; i < length; i++)
                {
                    //to DEC
                    buffer[i] = data[i] - 48;
                    //to Hex
                    result += buffer[i] * (pow(16,(length-i-1)));
                }
                return result;
            }
        private:
    };

}

#endif