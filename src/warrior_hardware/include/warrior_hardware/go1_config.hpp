#ifndef WARRIOR_HARDWARE_GO1_CONFIG_HPP_
#define WARRIOR_HARDWARE_GO1_CONFIG_HPP_
#include <string>
#include <vector>
namespace warrior_hardware
{
    struct SerialHdlcFrame
    {
        uint8_t data[17];
        size_t length;
    };

    class go1_config{
        public:
            go1_config();
            ~go1_config();
            int open(const std::string & port_name);
            int close();
            int read_frames(uint8_t* data, size_t size);
            int write_frame(const uint8_t* data, size_t size);
        private:
            int go1_port_;
            
    };
}
#endif