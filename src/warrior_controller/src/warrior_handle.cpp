
#include "warrior_controller/warrior_handle.hpp"
using namespace warrior_controller;

ImuHandle::ImuHandle(
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> pitch,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> yaw,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> roll,
    
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> wx,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> wy,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> wz,

    std::reference_wrapper<const hardware_interface::LoanedStateInterface> ax,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> ay,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> az
    ): pitch_(pitch),yaw_(yaw),roll_(roll),
        wx_(wx),wy_(wy),wz_(wz),
        ax_(ax),ay_(ay),az_(az)
{
}

LK9025Handle::LK9025Handle(
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> acceleration,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_ref,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_ref,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> torque_ref
    )
    : position_(position)
    , velocity_(velocity)
    , acceleration_(acceleration)
    , position_ref_(position_ref)
    , velocity_ref_(velocity_ref)
    , torque_ref_(torque_ref)
{
    
}

Go1Handle::Go1Handle(
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> acceleration,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_ref,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_ref,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> torque_ref,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> damp_ref,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> zero_torque_ref,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> torque_and_position_ref
    )
    : position_(position)
    , velocity_(velocity)
    , acceleration_(acceleration)
    , position_ref_(position_ref)
    , velocity_ref_(velocity_ref)
    , torque_ref_(torque_ref)
    , damp_ref_(damp_ref)
    , zero_torque_ref_(zero_torque_ref)
    , torque_and_position_ref_(torque_and_position_ref)
{
    
}
