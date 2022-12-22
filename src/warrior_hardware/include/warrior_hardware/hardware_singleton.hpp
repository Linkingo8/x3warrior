#ifndef WARRIOR_HARDWARE_SINGLETON_HPP_
#define WARRIOR_HARDWARE_SINGLETON_HPP_
#include <string>
#include <cstring>
#include <vector>

#include <iostream>
#include <string>

namespace warrior_hardware
{
class CanConfig {
    public:
        CanConfig(const CanConfig&) = delete;
        static CanConfig & get()
        {
            static CanConfig can_config;
            return can_config;
        }
        static uint8_t config_status(void) {return get().return_config_status();}
    private:
        CanConfig() = default;
        uint8_t return_config_status(void)
        {
            return can_config_status_;
        }
        uint8_t can_config_status_ = 0;
};
}
#endif