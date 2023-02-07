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
        static uint8_t get_status(void) {return get().return_config_status();}
        static uint8_t reverse_status(void) {get().reverse_config_status();}
    private:
        CanConfig() = default;
        uint8_t return_config_status(void)
        {
            return can_config_status_;
        }
        void reverse_config_status(void)
        {
            can_config_status_ = 1;
        }
        uint8_t can_config_status_ = 0;
};
}
#endif