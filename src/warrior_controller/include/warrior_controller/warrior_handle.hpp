#ifndef __WARRIOR_CONTROLLER__WARRIOR_HANDLE_H__
#define __WARRIOR_CONTROLLER__WARRIOR_HANDLE_H__
#include <hardware_interface/loaned_command_interface.hpp>
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


    class LK9025Handle
    {
        public:
            LK9025Handle(
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity,
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> acceleration,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_ref,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_ref,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> moment_ref
            );
            
            void set_position(double value){ position_ref_.get().set_value(value); };
            void set_velocity(double value){ velocity_ref_.get().set_value(value); };
            void set_moment(double value){ moment_ref_.get().set_value(value); };

            double get_position(){ return position_.get().get_value(); };
            double get_velocity(){ return velocity_.get().get_value(); };
            double get_acceleration(){ return acceleration_.get().get_value(); };

        private:
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_;
                std::reference_wrapper<const hardware_interface::LoanedStateInterface> acceleration_;
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_ref_;
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_ref_;
                std::reference_wrapper<hardware_interface::LoanedCommandInterface> moment_ref_;
    };
}
#endif // __WARRIOR_CONTROLLER__TEST_HANDLE_H__