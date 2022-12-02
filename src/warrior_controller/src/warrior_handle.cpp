
#include "warrior_controller/warrior_handle.hpp"

using namespace warrior_controller;

ImuHandle::ImuHandle(
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> pitch,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> yaw,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> roll
    ): pitch_(pitch),yaw_(yaw),roll_(roll)
{
}

