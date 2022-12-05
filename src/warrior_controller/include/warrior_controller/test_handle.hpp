#ifndef __WARRIOR_CONTROLLER__TEST_HANDLE_H__
#define __WARRIOR_CONTROLLER__TEST_HANDLE_H__

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>



namespace warrior_controller
{
    class TestHandle
    {
        public:
            TestHandle(
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> accelration,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_ref,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_ref,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> accelration_ref
                );

            void set_position(double value){ position_ref_.get().set_value(value); };
            void set_velocity(double value){ velocity_ref_.get().set_value(value); };
            void set_accelration(double value){ accelration_ref_.get().set_value(value); };

            double get_position(){ return position_.get().get_value(); };
            double get_velocity(){ return velocity_.get().get_value(); };
            double get_accelration(){ return accelration_.get().get_value(); };

        private:
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> accelration_;
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_ref_;
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_ref_;
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> accelration_ref_;

    };
}


#endif // __WARRIOR_CONTROLLER__TEST_HANDLE_H__