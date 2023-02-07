#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

//eigen
#include <iostream>
#include <Eigen/Dense>

#include "warrior_controller/wheel_balancing_controller.hpp"

using namespace warrior_controller;
/*?*/
WheelBalancingController::WheelBalancingController()
    : controller_interface::ControllerInterface()
    , command_subsciption_(nullptr)
    , command_ptr_(nullptr)
    , rc_commmonds_()
{
}

controller_interface::InterfaceConfiguration WheelBalancingController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    for(std::string joint : wheel_joint_name_)
    {
        command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
        command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
        command_interfaces_config.names.push_back(joint + "/" + "torque");
    }
    for(std::string joint : leg_joint_name_)
    {
        command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
        command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
        command_interfaces_config.names.push_back(joint + "/" + "torque");
        command_interfaces_config.names.push_back(joint + "/" + "damp");
        command_interfaces_config.names.push_back(joint + "/" + "zero_torque");
        command_interfaces_config.names.push_back(joint + "/" + "torque_and_position");
    }
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
    for(std::string joint : wheel_joint_name_)
    {
        state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
        state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
        state_interfaces_config.names.push_back(joint + "/" + "torque");
    }
    for(std::string joint : leg_joint_name_)
    {
        state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
        state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
        state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_ACCELERATION);
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
        auto_declare("joint2_name", "");
        auto_declare("joint3_name", "");
        auto_declare("joint_Go_LF_name", "");
        auto_declare("joint_Go_LB_name", "");
        auto_declare("joint_Go_RF_name", "");
        auto_declare("joint_Go_RB_name", "");
    }
    catch (const std::exception & e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
}

controller_interface::return_type WheelBalancingController::update()
{

    //feadback of LK and G01.

    //feadback of imu
    
    //update the remote date.
    WheelBalancingController::updatingRemoteData();

    RCLCPP_ERROR(get_node()->get_logger(), "remote rc->ch_l_x %f\n", 
        rc_commmonds_.ch_l_x);
    // RCLCPP_ERROR(get_node()->get_logger(), "remote rc->ch_l_y %f\n velocity %f\n", 
    //     rc->ch_l_y,lk_velocity);
    // RCLCPP_ERROR(get_node()->get_logger(), "remote rc->ch_l_y %f\n velocity %f\n", 
    //     rc->ch_r_x,lk_tourque);
    
    if(rc_commmonds_.sw_l == 1) { //protection mode
        LK_L_handles_->set_torque(0);
        LK_R_handles_->set_torque(0);
        Go1_LF_handles_->set_torque(0);
        Go1_LB_handles_->set_torque(0);
        Go1_RF_handles_->set_torque(0);
        Go1_RB_handles_->set_torque(0);
    } else {//other modes
        LK_L_handles_->set_torque(0);
        LK_R_handles_->set_torque(0);
        Go1_LF_handles_->set_torque(0);
        Go1_LB_handles_->set_torque(0);
        Go1_RF_handles_->set_torque(0);
        Go1_RB_handles_->set_torque(0);
    }
     return controller_interface::return_type::OK;
}

CallbackReturn WheelBalancingController::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_node()->get_logger(), "Configure WheelBalancingController");

    auto joint1_name_ = get_node()->get_parameter("joint1_name").as_string();
    auto joint2_name_ = get_node()->get_parameter("joint2_name").as_string();
    auto joint3_name_ = get_node()->get_parameter("joint3_name").as_string();

    auto joint_Go_LF_name_ = get_node()->get_parameter("joint_Go_LF_name").as_string();
    auto joint_Go_LB_name_ = get_node()->get_parameter("joint_Go_LB_name").as_string();
    auto joint_Go_RF_name_ = get_node()->get_parameter("joint_Go_RF_name").as_string();
    auto joint_Go_RB_name_ = get_node()->get_parameter("joint_Go_RB_name").as_string();

    if (joint1_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'joint1_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    imu_joint_name_.push_back(joint1_name_);

    if (joint2_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'joint2_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    wheel_joint_name_.push_back(joint2_name_);

    if (joint3_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'joint3_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    wheel_joint_name_.push_back(joint3_name_);

    if (joint_Go_LF_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'joint4_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    leg_joint_name_.push_back(joint_Go_LF_name_);

    if (joint_Go_LB_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'joint5_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    leg_joint_name_.push_back(joint_Go_LB_name_);

        if (joint_Go_RF_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'joint6_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    leg_joint_name_.push_back(joint_Go_RF_name_);

        if (joint_Go_RB_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'joint7_name' parameter was empty");
        return CallbackReturn::ERROR;
    }
    leg_joint_name_.push_back(joint_Go_RB_name_);

    command_subsciption_ = get_node()->create_subscription<warrior_interface::msg::DbusData>("/rc_msg", 10, [this](const std::shared_ptr<warrior_interface::msg::DbusData> rc)
    {
        command_ptr_.writeFromNonRT(rc);
    });

    return CallbackReturn::SUCCESS;
}

CallbackReturn WheelBalancingController::on_activate(const rclcpp_lifecycle::State &)
{
    // Initialize the joint handle
    imu_handles_ = get_angle(imu_joint_name_.at(0));
    LK_L_handles_  = get_LK_handle(wheel_joint_name_.at(0));
    LK_R_handles_ =  get_LK_handle(wheel_joint_name_.at(1));

    Go1_LF_handles_ =  get_Go1_handle(leg_joint_name_.at(0));
    Go1_LB_handles_ =  get_Go1_handle(leg_joint_name_.at(1));
    Go1_RF_handles_ =  get_Go1_handle(leg_joint_name_.at(2));
    Go1_RB_handles_ =  get_Go1_handle(leg_joint_name_.at(3));

    if (!imu_handles_) {
        return CallbackReturn::ERROR;
    }
    if (!LK_L_handles_) {
        return CallbackReturn::ERROR;
    }
    if (!LK_R_handles_) {
        return CallbackReturn::ERROR;
    }
    if (!Go1_LF_handles_ || !Go1_LB_handles_ || !Go1_RF_handles_ || !Go1_RB_handles_) {
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

//get remote date
controller_interface::return_type WheelBalancingController::updatingRemoteData(void)
{
    // look the subcription
    auto command = command_ptr_.readFromRT();
    if (!command || !(*command)) {
        return controller_interface::return_type::OK;
    }
    const auto rc = (*command);
    //load to private remote variable.
     rc_commmonds_.ch_l_x  = rc->ch_l_x   * 1000;
     rc_commmonds_.ch_l_y  = rc->ch_l_y   * 1000;
     rc_commmonds_.ch_r_x  = rc->ch_r_x   * 1000;
     rc_commmonds_.ch_r_y  = rc->ch_r_y   * 1000;
     rc_commmonds_.sw_l = rc->s_l;
     rc_commmonds_.sw_r = rc->s_r;
     rc_commmonds_.wheel = rc->wheel;
    return controller_interface::return_type::OK;
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

std::shared_ptr<LK9025Handle> WheelBalancingController::get_LK_handle(const std::string & joint_name)
{
    // Lookup the pitch state interface
    const auto position_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
    });
    if (position_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s position state interface not found", joint_name.c_str());
        return nullptr;
    }

    // Lookup the yaw state interface
    const auto velocity_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity state interface not found", joint_name.c_str());
        return nullptr;
    }

    // Lookup the roll state interface
    const auto acceleration_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == "torque";
    });
    if (acceleration_state == state_interfaces_.cend()) {
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
    const auto torque_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == "torque";
    });
    if (torque_command == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s accelration command interface not found", joint_name.c_str());
        return nullptr;
    }

    // Create the ImuHandle instance
    return std::make_shared<LK9025Handle>(std::ref(*position_state),
                                        std::ref(*velocity_state),
                                        std::ref(*acceleration_state),
                                        std::ref(*position_command),
                                        std::ref(*velocity_command),
                                        std::ref(*torque_command));
}

std::shared_ptr<Go1Handle> WheelBalancingController::get_Go1_handle(const std::string & joint_name)
{
    // Lookup the pitch state interface
    const auto position_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
    });
    if (position_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s position state interface not found", joint_name.c_str());
        return nullptr;
    }

    // Lookup the yaw state interface
    const auto velocity_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity state interface not found", joint_name.c_str());
        return nullptr;
    }

    // Lookup the roll state interface
    const auto acceleration_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == hardware_interface::HW_IF_ACCELERATION;
    });
    if (acceleration_state == state_interfaces_.cend()) {
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

    // Lookup the motion command interface
    const auto torque_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == "torque";
    });
    if (torque_command == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s motion command interface not found", joint_name.c_str());
        return nullptr;
    }

    // Lookup the damp command interface
    const auto damp_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == "damp";
    });
    if (damp_command == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s damp command interface not found", joint_name.c_str());
        return nullptr;
    }
    // Lookup the zero motion command interface
    const auto zero_torque_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == "zero_torque";
    });
    if (zero_torque_command == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s zero motion command interface not found", joint_name.c_str());
        return nullptr;
    }
    // Lookup the motion command interface
    const auto torque_and_position_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == "torque_and_position";
    });
    if (torque_and_position_command == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s position and motion command interface not found", joint_name.c_str());
        return nullptr;
    }

    // Create the ImuHandle instance
    return std::make_shared<Go1Handle>(std::ref(*position_state),
                                        std::ref(*velocity_state),
                                        std::ref(*acceleration_state),
                                        std::ref(*position_command),
                                        std::ref(*velocity_command),
                                        std::ref(*torque_command),
                                        std::ref(*damp_command),
                                        std::ref(*zero_torque_command),
                                        std::ref(*torque_and_position_command));
}

PLUGINLIB_EXPORT_CLASS(
    warrior_controller::WheelBalancingController,
    controller_interface::ControllerInterface
)