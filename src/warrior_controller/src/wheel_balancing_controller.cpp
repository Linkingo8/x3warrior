#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include "warrior_controller/wheel_balancing_controller.hpp"


using namespace warrior_controller;
/*?*/
WheelBalancingController::WheelBalancingController()
    : controller_interface::ControllerInterface()
    , command_subsciption_(nullptr)
    , command_ptr_(nullptr)
    , imu_data_publisher_(nullptr)
    , realtime_imu_data_publisher_(nullptr)
    , LK9025_data_publisher_(nullptr)
    , realtime_LK9025_data_publisher_(nullptr)
    , Go1_data_publisher_(nullptr)
    , realtime_Go1_data_publisher_(nullptr)
    , rc_commmonds_()
    , A_(6, 6)
    , B_(6, 2)
    , Q_(6, 6)
    , R_(2, 2)
    , K_(2, 6)
    , P_(2, 2)
    , lqr_(nullptr)
    , left_destination_()
    , leg_balance_controller_()
    , pitch_now_(0.0)
    , pitch_last_(0.0)

{ 
        A_ <<   0,              0,          0,          0,          4.900,          -4.900,
                0,              0,          0,          0,          -40.6842,       -40.6842,
                0,              0,          0,          0,          -197.900,       -197.900,
                1.0000,         0,          0,          0,          0,               0 ,
                0,              1.0000,     0,          0,          0,               0,
                0,              0,          1.0000,     0,          0,               0;
        B_ <<   -1.9887,     -2.7600,
                16.5121,     -34.7769,  
                -22.6242,    -169.1604, 
                0,           0, 
                0,           0, 
                0,           0;
        Q_ <<   1,              0,          0,          0,          0,               0,
                0,              1,          0,          0,          0,               0,
                0,              0,          1,          0,          0,               0,
                0,              0,          0,          1,          0,               0,
                0,              0,          0,          0,          1,               0,
                0,              0,          0,          0,          0,               1; 
        R_ <<   1,   0,
                0,   1; 

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
    // x_dot = Ax + Bu
    /// x[x x_dot theta theta_dot fai fai_dot]
    /// x[驱动轮位移 驱动轮加速度 关节电机位移 关节电机加速度 陀螺仪 陀螺仪加速度]
    /// define state-vector A;

    // u = k(x_d - x)
    // u = [tau_W,tau_B]

    // k = lqr_ - > k       [*]

    // x_d [x,  theta,  fai,    x_dot,  theta_dot,    fai_dot]_des  [*]
    // x_d [0,  theta,  0,      0,      0,                  0]_des  [*]

    // x[lk_pos,theta_cal,imu_pitch,lk_velocities,theta_cal_dot,imu_pitch_dot]
    

    ///Right-handed coordinate systgem
    /// for go1     id:0    id: 1 id: 2 id :3
    /// torque      + f     + b   + b   + f
    /// for lk9025  id:1  id: 2
    /// torque

    //feadback of LK and G01.
 
    //feadback of imu
    
    //update the remote date.
    WheelBalancingController::updatingRemoteData();
    /// calculate the lqr k.
    lqr_->K = lqr_->calcGainK();
    /// set the x_d to struct
    WheelBalancingController::updateXdes(0);
    /// get the feedback struct
    WheelBalancingController::updateX(0);
    /// give k to controller
    WheelBalancingController::setLegLQRGain(lqr_->K,0);
    /// set the x_d to controller
    WheelBalancingController::setLegLQRXd(0);
    /// set the X to controller
    WheelBalancingController::setLegLQRX(0);
    WheelBalancingController::calclegLQRU(0);

    /// set the line input to track the root of system
    if(rc_commmonds_.sw_l == 1) { //protection mode
        LK_L_handles_->set_torque(0);
        LK_R_handles_->set_torque(0);
        /// left leg //     + - up       - + down
        Go1_LF_handles_->set_torque(0);
        Go1_LB_handles_->set_torque(0);
        /// right leg //    - + up       + - down
        Go1_RF_handles_->set_torque(0);
        Go1_RB_handles_->set_torque(0);
    } else {//other modes
        LK_L_handles_->set_torque(0);
        LK_R_handles_->set_torque(0);
        /// left leg
        Go1_LF_handles_->set_torque(0.0f);
        Go1_LB_handles_->set_torque(0.0f);
        /// right leg
        Go1_RF_handles_->set_torque(0.0f);
        Go1_RB_handles_->set_torque(0.0f);
    }
/// publish sensor feedback.
#ifdef IMU_PLOT
    if (realtime_imu_data_publisher_->trylock())
    {
      auto & imu_data_message = realtime_imu_data_publisher_->msg_;
      imu_data_message.pitch = imu_handles_->get_pitch();
      imu_data_message.yaw = imu_handles_->get_yaw();
      imu_data_message.roll = imu_handles_->get_roll();
      realtime_imu_data_publisher_->unlockAndPublish();
    }
#endif

#ifdef LK_PLOT
    if (realtime_LK9025_data_publisher_->trylock())
    {
      auto & lk9025_data_message = realtime_LK9025_data_publisher_->msg_;
      lk9025_data_message.lpositions = LK_L_handles_->get_position();
      lk9025_data_message.lvelocities = LK_L_handles_->get_velocity();
      lk9025_data_message.ltorques = LK_L_handles_->get_acceleration();
      lk9025_data_message.rpositions = LK_R_handles_->get_position();
      lk9025_data_message.rvelocities = LK_R_handles_->get_velocity();
      lk9025_data_message.rtorques = LK_R_handles_->get_acceleration();
      realtime_LK9025_data_publisher_->unlockAndPublish();
    }
#endif

#ifdef GO1_PLOT
    if (realtime_Go1_data_publisher_->trylock())
    {
      auto & Go1_data_message = realtime_Go1_data_publisher_->msg_;
      Go1_data_message.lfpositions = Go1_LF_handles_->get_position();
      Go1_data_message.lfvelocities = Go1_LF_handles_->get_velocity();
      Go1_data_message.lftorques = Go1_LF_handles_->get_acceleration();

      Go1_data_message.rfpositions = Go1_RF_handles_->get_position();
      Go1_data_message.rfvelocities = Go1_RF_handles_->get_velocity();
      Go1_data_message.rftorques = Go1_RF_handles_->get_acceleration();

      Go1_data_message.rbpositions = Go1_RB_handles_->get_position();
      Go1_data_message.rbvelocities = Go1_RB_handles_->get_velocity();
      Go1_data_message.rbtorques = Go1_RB_handles_->get_acceleration();
      
      Go1_data_message.lbpositions = Go1_LB_handles_->get_position();
      Go1_data_message.lbvelocities = Go1_LB_handles_->get_velocity();
      Go1_data_message.lbtorques = Go1_LB_handles_->get_acceleration();
      realtime_Go1_data_publisher_->unlockAndPublish();
    }

#endif
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

    lqr_ = std::make_shared<LQR>();
    WheelBalancingController::initLQRParam();

#ifdef IMU_PLOT
    // initialize transform publisher and message
    imu_data_publisher_ = get_node()->create_publisher<warrior_interface::msg::ImuData>("/imu_feedback", 
        rclcpp::SystemDefaultsQoS());
    realtime_imu_data_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<warrior_interface::msg::ImuData>>(
            imu_data_publisher_);
    auto & imu_data_message = realtime_imu_data_publisher_->msg_;
    imu_data_message.pitch = 0.00;
    imu_data_message.yaw = 0.00;
    imu_data_message.roll = 0.00;
#endif

#ifdef LK_PLOT
    // initialize transform publisher and message
    LK9025_data_publisher_ = get_node()->create_publisher<warrior_interface::msg::LK9025Feedback>("/lk9025_feedback", 
        rclcpp::SystemDefaultsQoS());
    realtime_LK9025_data_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<warrior_interface::msg::LK9025Feedback>>(
            LK9025_data_publisher_);
    auto & lk9025_data_message = realtime_LK9025_data_publisher_->msg_;
    lk9025_data_message.lpositions = 0.00;
    lk9025_data_message.lvelocities = 0.00;
    lk9025_data_message.ltorques = 0.00;
    lk9025_data_message.rpositions = 0.00;
    lk9025_data_message.rvelocities = 0.00;
    lk9025_data_message.rtorques = 0.00;
#endif

#ifdef GO1_PLOT
    // initialize transform publisher and message
    Go1_data_publisher_ = get_node()->create_publisher<warrior_interface::msg::Go1Feedback>("/go1_feedback", 
        rclcpp::SystemDefaultsQoS());
    realtime_Go1_data_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<warrior_interface::msg::Go1Feedback>>(
            Go1_data_publisher_);
    auto & Go1_data_message = realtime_Go1_data_publisher_->msg_;
    Go1_data_message.lfpositions = 0.00;
    Go1_data_message.lfvelocities = 0.00;
    Go1_data_message.lftorques = 0.00;

    Go1_data_message.rfpositions = 0.00;
    Go1_data_message.rfvelocities = 0.00;
    Go1_data_message.rftorques = 0.00;

    Go1_data_message.rbpositions = 0.00;
    Go1_data_message.rbvelocities = 0.00;
    Go1_data_message.rbtorques = 0.00;

    Go1_data_message.lbpositions = 0.00;
    Go1_data_message.lbvelocities = 0.00;
    Go1_data_message.lbtorques = 0.00;
#endif

    return CallbackReturn::SUCCESS;
}

CallbackReturn WheelBalancingController::on_activate(const rclcpp_lifecycle::State &)
{
    // Initialize the joint handle
    imu_handles_ = get_angle(imu_joint_name_.at(0));

    LK_L_handles_  = get_LK_handle(wheel_joint_name_.at(0));
    LK_R_handles_ =  get_LK_handle(wheel_joint_name_.at(1));

    Go1_LF_handles_ =  get_Go1_handle(leg_joint_name_.at(0));
    Go1_RF_handles_ =  get_Go1_handle(leg_joint_name_.at(1));
    Go1_RB_handles_ =  get_Go1_handle(leg_joint_name_.at(2));
    Go1_LB_handles_ =  get_Go1_handle(leg_joint_name_.at(3));

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
/// banlance controller
/// 0 left 1 right 
void WheelBalancingController::setLegLQRGain(MatrixXd K,uint8_t index)
{
    leg_balance_controller_[index].K = K;
    // std::cout << leg_balance_controller_[index].K;
}

void WheelBalancingController::updateXdes(uint8_t index)
{
    if(index == 0)
    {
        left_destination_.x = 0.0f;
        left_destination_.x_dot = 0.0f;
        left_destination_.theta = 0.0f;
        left_destination_.theta_dot = 0.0f;
        left_destination_.fai = 0.0f;
        left_destination_.fai_dot = 0.0f;
    }
    if(index == 1)
    {
        left_destination_.x = 0;
        left_destination_.x_dot = 0;
        left_destination_.theta = 0;
        left_destination_.theta_dot = 0;
        left_destination_.fai = 0;
        left_destination_.fai_dot = 0;
    }
}

void WheelBalancingController::setLegLQRXd(uint8_t index)
{
    if(index == 0)
    {
        leg_balance_controller_[index].X_d << left_destination_.x
        ,left_destination_.x_dot
        ,left_destination_.theta
        ,left_destination_.theta_dot
        ,left_destination_.fai
        ,left_destination_.fai_dot;
        // std::cout << leg_balance_controller_[index].X_d << std::endl;
    }
    if(index == 1)
    {
        // leg_balance_controller_[index].X_d(1,1) = left_destination_.x;
        // leg_balance_controller_[index].X_d(1,2) = left_destination_.x_dot;
        // leg_balance_controller_[index].X_d(1,3) = left_destination_.theta;
        // leg_balance_controller_[index].X_d(1,4) = left_destination_.theta_dot;
        // leg_balance_controller_[index].X_d(1,5) = left_destination_.fai;
        // leg_balance_controller_[index].X_d(1,6) = left_destination_.fai_dot;
    }
}

void WheelBalancingController::updateX(uint8_t index)
{
    pitch_now_ = imu_handles_->get_pitch();
        if(index == 0)
    {
        left_set_feedback_.x = LK_L_handles_->get_position();
        left_set_feedback_.x_dot = LK_L_handles_->get_velocity();
        left_set_feedback_.theta = 0.0f;
        left_set_feedback_.theta_dot = 0.0f;
        left_set_feedback_.fai = pitch_now_;
        left_set_feedback_.fai_dot = pitch_now_ - pitch_last_;
    }
    if(index == 1)
    {
        left_set_feedback_.x = 0;
        left_set_feedback_.x_dot = 0;
        left_set_feedback_.theta = 0;
        left_set_feedback_.theta_dot = 0;
        left_set_feedback_.fai = 0;
        left_set_feedback_.fai_dot = 0;
    }
    pitch_last_ = pitch_now_;
}

void WheelBalancingController::setLegLQRX(uint8_t index)
{
    if(index == 0)
    {
        leg_balance_controller_[index].X << left_set_feedback_.x
        ,left_set_feedback_.x_dot
        ,left_set_feedback_.theta
        ,left_set_feedback_.theta_dot
        ,left_set_feedback_.fai
        ,left_set_feedback_.fai_dot;
        // std::cout << leg_balance_controller_[index].X << std::endl;
    }
    if(index == 1)
    {
    }    
}
void WheelBalancingController::calclegLQRU(uint8_t index)
{
    if(index == 0)
    {
        leg_balance_controller_[index].U = leg_balance_controller_[index].K * 
                    (leg_balance_controller_[index].X_d.transpose() - leg_balance_controller_[index].X.transpose());
        // std::cout << leg_balance_controller_[index].U << std::endl;
        
    }
    if(index == 1)
    {
    }        
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

void WheelBalancingController::initLQRParam(void)
{
        lqr_ -> A = A_;
        lqr_ -> B = B_;
        lqr_ -> Q = Q_;
        lqr_ -> R = R_;
        lqr_ -> K = K_;
        lqr_ -> P = P_;
        // std::cout <<  lqr_ -> A << std::endl;
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