
#include "warrior_controller/test_handle.hpp"

using namespace warrior_controller;

TestHandle::TestHandle(
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> accelration,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_ref,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_ref,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> accelration_ref   
    )
    : position_(position)
    , velocity_(velocity)
    , accelration_(accelration)
    , position_ref_(position_ref)
    , velocity_ref_(velocity_ref)
    , accelration_ref_(accelration_ref)
{
    
}

