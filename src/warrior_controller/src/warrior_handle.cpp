
#include "warrior_controller/warrior_handle.hpp"
using namespace warrior_controller;

ImuHandle::ImuHandle(
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> pitch,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> yaw,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> roll
    ): pitch_(pitch),yaw_(yaw),roll_(roll)
{
}

LK9025Handle::LK9025Handle(
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> acceleration,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_ref,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_ref,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> moment_ref
    )
    : position_(position)
    , velocity_(velocity)
    , acceleration_(acceleration)
    , position_ref_(position_ref)
    , velocity_ref_(velocity_ref)
    , moment_ref_(moment_ref)
{
    
}

Go1Handle::Go1Handle(
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> acceleration,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_ref,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_ref,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> moment_ref
    )
    : position_(position)
    , velocity_(velocity)
    , acceleration_(acceleration)
    , position_ref_(position_ref)
    , velocity_ref_(velocity_ref)
    , moment_ref_(moment_ref)
{
    
}
