
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "warrior_controller/test_controller.hpp"

using namespace warrior_controller;

TestController::TestController()
    : controller_interface::ControllerInterface()
    , command_subsciption_(nullptr)
    , command_ptr_(nullptr)
{
}

controller_interface::InterfaceConfiguration TestController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for(std::string joint : test_joint_name_)
    {
        command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
        command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
        command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_ACCELERATION);
    }

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration TestController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;


    for(std::string joint : test_joint_name_)
    {
        state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
        state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
        state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_ACCELERATION);
    }

    return state_interfaces_config;
}

controller_interface::return_type TestController::init(const std::string & controller_name)
{
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK) {
        return ret;
    }

    try {
        auto node = get_node();
        auto_declare("joint1_name", "");
        auto_declare("joint2_name", "");
    }
    catch (const std::exception & e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
}

controller_interface::return_type TestController::update()
{
    // Get the last velocity command
    auto command = command_ptr_.readFromRT();
    if (!command || !(*command)) {
        return controller_interface::return_type::OK;
    }
    // RCLCPP_ERROR(get_node()->get_logger(), "test_joint1 position %f velocity %f accelration %f", test_joint1_handles_->get_position()
    // ,test_joint1_handles_->get_velocity(),test_joint1_handles_->get_accelration());

    // RCLCPP_ERROR(get_node()->get_logger(), "test_joint2 position %f velocity %f accelration %f", test_joint2_handles_->get_position()
    // ,test_joint2_handles_->get_velocity(),test_joint2_handles_->get_accelration());
    const auto rc = (*command);
    //RCLCPP_ERROR(get_node()->get_logger(), "rc -> sw1 %d ",rc->s_l);
    double position  = rc->ch_l_x + 100.0;
    double velocity  = rc->ch_l_y + 200.0;
    double accelration  = rc->ch_l_x + rc->ch_l_y + 300.0;


    test_joint1_handles_->set_position(position);
    test_joint1_handles_->set_velocity(velocity);
    test_joint1_handles_->set_accelration(accelration);

    test_joint2_handles_->set_position(position);
    test_joint2_handles_->set_velocity(velocity);
    test_joint2_handles_->set_accelration(accelration);

    return controller_interface::return_type::OK;
}

CallbackReturn TestController::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_node()->get_logger(), "Configure TestController");

    auto joint1_name_ = get_node()->get_parameter("joint1_name").as_string();
    auto joint2_name_ = get_node()->get_parameter("joint2_name").as_string();


    if (joint1_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'joint1_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    if (joint2_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'joint2_name' parameter was empty");
        return CallbackReturn::ERROR;
    }

    test_joint_name_.push_back(joint1_name_);
    test_joint_name_.push_back(joint2_name_);

    command_subsciption_ = get_node()->create_subscription<warrior_interface::msg::DbusData>("/rc_msg", 10, [this](const std::shared_ptr<warrior_interface::msg::DbusData> rc)
    {
        command_ptr_.writeFromNonRT(rc);
    });

    return CallbackReturn::SUCCESS;
}

CallbackReturn TestController::on_activate(const rclcpp_lifecycle::State &)
{
    // Initialize the joint handle
    test_joint1_handles_ = get_handle(test_joint_name_.at(0));
    test_joint2_handles_ = get_handle(test_joint_name_.at(1));

    if (!test_joint1_handles_ || !test_joint2_handles_ ) {
        return CallbackReturn::ERROR;
    }
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn TestController::on_deactivate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn TestController::on_cleanup(const rclcpp_lifecycle::State &)
{

    return CallbackReturn::SUCCESS;
}

CallbackReturn TestController::on_error(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn TestController::on_shutdown(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

std::shared_ptr<TestHandle> TestController::get_handle(const std::string & joint_name)
{
    // Lookup the position state interface
    const auto position_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
    });
    if (position_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s position state interface not found", joint_name.c_str());
        return nullptr;
    }

    // Lookup the velocity state interface
    const auto velocity_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity state interface not found", joint_name.c_str());
        return nullptr;
    }

    // Lookup the accelration state interface
    const auto accelration_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == hardware_interface::HW_IF_ACCELERATION;
    });
    if (accelration_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s accelration state interface not found", joint_name.c_str());
        return nullptr;
    }

    // Lookup the position command interface
    const auto position_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
    });
    if (position_command == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s position command interface not found", joint_name.c_str());
        return nullptr;
    }


    // Lookup the velocity command interface
    const auto velocity_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_command == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity command interface not found", joint_name.c_str());
        return nullptr;
    }

    // Lookup the accelration command interface
    const auto accelration_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == hardware_interface::HW_IF_ACCELERATION;
    });
    if (accelration_command == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s accelration command interface not found", joint_name.c_str());
        return nullptr;
    }

    // Create the TestHandle instance

    return std::make_shared<TestHandle>(std::ref(*position_state),
                                        std::ref(*velocity_state),
                                        std::ref(*accelration_state),
                                        std::ref(*position_command),
                                        std::ref(*velocity_command),
                                        std::ref(*accelration_command));
}


PLUGINLIB_EXPORT_CLASS(
    warrior_controller::TestController,
    controller_interface::ControllerInterface
)