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
class Debugsig {
    public:
        Debugsig(const CanConfig&) = delete;
        static Debugsig & get()
        {
            static Debugsig debug_sig;
            return debug_sig;
        }
        static double get_go_start(void) {return get().get_go_start_();}
        static void set_go_start(double start_time) { get().set_go_start_(start_time);}
        
        static double get_go_end(void) {return get().get_go_end_();}
        static void set_go_end(double end_time) { get().set_go_end_(end_time);}

        static uint8_t get_go_flag(void) { return get().get_go_flag_();}
        static void set_go_flag(uint8_t flag) { get().set_flag_(flag);}

    private:
        Debugsig() = default;
        // start time
        double get_go_start_(void)
        {
            return go_start_;
        }
        void set_go_start_(double start_time)
        {
            go_start_ = start_time;
        }
        // end time
        double get_go_end_(void)
        {
            return go_end_;
        }
        void set_go_end_(double end_time)
        {
            go_end_ = end_time;
        }

        void set_flag_(uint8_t flag)
        {
            go_flag_ = flag;
        }
        uint8_t get_go_flag_(void)
        {
            return go_flag_;
        }
        double go_start_,go_end_,go_duration_;
        uint8_t go_flag_ = 0;
};
}
#endif