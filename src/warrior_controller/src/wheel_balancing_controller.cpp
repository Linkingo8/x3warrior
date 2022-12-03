
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "warrior_controller/wheel_balancing_controller.hpp"

using namespace warrior_controller;
/*?*/
WheelBalancingController::WheelBalancingController()
    : controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration WheelBalancingController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration WheelBalancingController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;


    for(std::string joint : imu_joint_name_)
    {
        state_interfaces_config.names.push_back(joint + "/" + "pitch");
        state_interfaces_config.names.push_back(joint + "/" + "yaw");
        state_interfaces_config.names.push_back(joint + "/" + "roll");
    }

    return state_interfaces_config;
}

controller_interface::return_type WheelBalancingController::init(const std::string & controller_name)
{
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK) {
        return ret;
    }

    try {
        auto node = get_node();
        auto_declare("joint1_name", "");
    }
    catch (const std::exception & e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
}

controller_interface::return_type WheelBalancingController::update()
{
    // RCLCPP_ERROR(get_node()->get_logger(), 
    //     "imu_joint pitch %f yaw %f roll %f"
    //     ,imu_handles_->get_pitch()
    //     ,imu_handles_->get_yaw()
    //     ,imu_handles_->get_roll());
     return controller_interface::return_type::OK;
}

CallbackReturn WheelBalancingController::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_node()->get_logger(), "Configure WheelBalancingController");

    auto joint1_name_ = get_node()->get_parameter("joint1_name").as_string();
    if (joint1_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'joint1_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    imu_joint_name_.push_back(joint1_name_);

    return CallbackReturn::SUCCESS;
}

CallbackReturn WheelBalancingController::on_activate(const rclcpp_lifecycle::State &)
{
    // Initialize the joint handle
    imu_handles_ = get_angle(imu_joint_name_.at(0));

    if (!imu_handles_) {
        return CallbackReturn::ERROR;
    }
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn WheelBalancingController::on_deactivate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn WheelBalancingController::on_cleanup(const rclcpp_lifecycle::State &)
{

    return CallbackReturn::SUCCESS;
}

CallbackReturn WheelBalancingController::on_error(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn WheelBalancingController::on_shutdown(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

std::shared_ptr<ImuHandle> WheelBalancingController::get_angle(const std::string & joint_name)
{
    // Lookup the pitch state interface
    const auto pitch_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == "pitch";
    });
    if (pitch_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s position state interface not found", joint_name.c_str());
        return nullptr;
    }

    // Lookup the yaw state interface
    const auto yaw_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == "yaw";
    });
    if (yaw_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity state interface not found", joint_name.c_str());
        return nullptr;
    }

    // Lookup the roll state interface
    const auto roll_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == "roll";
    });
    if (roll_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s accelration state interface not found", joint_name.c_str());
        return nullptr;
    }
    // Create the ImuHandle instance

    return std::make_shared<ImuHandle>(std::ref(*pitch_state),
                                        std::ref(*yaw_state),
                                        std::ref(*roll_state));
}


PLUGINLIB_EXPORT_CLASS(
    warrior_controller::WheelBalancingController,
    controller_interface::ControllerInterface
)