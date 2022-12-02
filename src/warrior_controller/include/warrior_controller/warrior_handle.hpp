#ifndef __WARRIOR_CONTROLLER__IMU_HANDLE_H__
#define __WARRIOR_CONTROLLER__IMU_HANDLE_H__

#include <hardware_interface/loaned_state_interface.hpp>
namespace warrior_controller
{
    class ImuHandle
    {
        public:
            ImuHandle(
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> pitch,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> yaw,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> roll
                );

            double get_pitch(){ return pitch_.get().get_value(); };
            double get_yaw(){ return yaw_.get().get_value(); };
            double get_roll(){ return roll_.get().get_value(); };

        private:
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> pitch_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> yaw_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> roll_;

    };
}
#endif // __WARRIOR_CONTROLLER__TEST_HANDLE_H__