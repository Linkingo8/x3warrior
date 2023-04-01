#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include "warrior_controller/wheel_balancing_controller.hpp"

using namespace warrior_controller;
/*?*/
WheelBalancingController::WheelBalancingController()
    : controller_interface::ControllerInterface()
    , simu_leg_lb_p_subsciption_(nullptr)
    , simu_leg_lb_v_subsciption_(nullptr)
    , simu_leg_lf_p_subsciption_(nullptr)
    , simu_leg_lf_v_subsciption_(nullptr)
    , simu_leg_rb_p_subsciption_(nullptr)
    , simu_leg_rb_v_subsciption_(nullptr)
    , simu_leg_rf_p_subsciption_(nullptr)
    , simu_leg_rf_v_subsciption_(nullptr)
    , simu_wheelleft_p_subsciption_(nullptr)
    , simu_wheelleft_v_subsciption_(nullptr)
    , simu_wheelright_p_subsciption_(nullptr)
    , simu_wheelright_v_subsciption_(nullptr)
    , simu_imu_subsciption_(nullptr)
    , simu_leg_lb_p_ptr_(nullptr)
    , simu_leg_lb_v_ptr_(nullptr)
    , simu_leg_lf_p_ptr_(nullptr)
    , simu_leg_lf_v_ptr_(nullptr)
    , simu_leg_rb_p_ptr_(nullptr)
    , simu_leg_rb_v_ptr_(nullptr)
    , simu_leg_rf_p_ptr_(nullptr)
    , simu_leg_rf_v_ptr_(nullptr)
    , simu_wheelleft_p_ptr_(nullptr)
    , simu_wheelleft_v_ptr_(nullptr)
    , simu_wheelright_p_ptr_(nullptr)
    , simu_wheelright_v_ptr_(nullptr)
    , simu_imu_ptr_(nullptr)
    , command_subsciption_(nullptr)
    , command_ptr_(nullptr)
    , imu_data_publisher_(nullptr)
    , realtime_imu_data_publisher_(nullptr)
    , LK9025_data_publisher_(nullptr)
    , realtime_LK9025_data_publisher_(nullptr)
    , Go1_data_publisher_(nullptr)
    , realtime_Go1_data_publisher_(nullptr)
    , VMC_debug_data_publisher_(nullptr) 
    , realtime_VMC_debug_data_publisher_(nullptr)
    , LQR_debug_data_publisher_(nullptr)
    , realtime_LQR_debug_data_publisher_(nullptr)
    , Simulation_data_publisher_(nullptr)
    , realtime_Simulation_data_publisher_(nullptr)
    , rc_commmonds_()
    , left_lqr_(nullptr)
    , right_lqr_(nullptr)
    , lqr_(nullptr)
    , left_five_bar_(nullptr)
    , left_vmc_(nullptr)
    , left_Fy_pid_(nullptr)
    , left_destination_()
    , left_set_feedback_()
    , state_var_tar_()
    , state_var_now_()
    , leg_balance_controller_()
    , pitch_now_(0.0)
    , pitch_last_(0.0)
{    
        left_leg_lqr_param_.A_ <<   0,              0,          0,          0,          4.900,          -4.900,
                0,              0,          0,          0,          -40.6842,       -40.6842,
                0,              0,          0,          0,          -197.900,       -197.900,
                1.0000,         0,          0,          0,          0,               0 ,
                0,              1.0000,     0,          0,          0,               0,
                0,              0,          1.0000,     0,          0,               0;
        left_leg_lqr_param_.B_ <<   -1.9887,     -2.7600,
                16.5121,     -34.7769,  
                -22.6242,    -169.1604, 
                0,           0, 
                0,           0, 
                0,           0;
        left_leg_lqr_param_.Q_ <<   1,              0,          0,          0,          0,               0,
                0,              1,          0,          0,          0,               0,
                0,              0,          1,          0,          0,               0,
                0,              0,          0,          1,          0,               0,
                0,              0,          0,          0,          1,               0,
                0,              0,          0,          0,          0,               1; 
        left_leg_lqr_param_.R_ <<   1,    0,
                0,      1; 

        right_leg_lqr_param_.A_ <<   0,              0,          0,          0,          4.900,          -4.900,
                0,              0,          0,          0,          -40.6842,       -40.6842,
                0,              0,          0,          0,          -197.900,       -197.900,
                1.0000,         0,          0,          0,          0,               0 ,
                0,              1.0000,     0,          0,          0,               0,
                0,              0,          1.0000,     0,          0,               0;
        right_leg_lqr_param_.B_ <<   -1.9887,     -2.7600,
                16.5121,     -34.7769,  
                -22.6242,    -169.1604, 
                0,           0, 
                0,           0, 
                0,           0;
        right_leg_lqr_param_.Q_ <<   1,              0,          0,          0,          0,               0,
                0,              1,          0,          0,          0,               0,
                0,              0,          1,          0,          0,               0,
                0,              0,          0,          1,          0,               0,
                0,              0,          0,          0,          1,               0,
                0,              0,          0,          0,          0,               1; 
        right_leg_lqr_param_.R_ <<   1,    0,
                0,      1; 

        leg_lqr_param_.A_ <<        
    1.0000,    0.0000,   -0.0000,         0,    0.0098,   -0.0098,
         0,    0.9999,   -0.0001,         0,   -0.0814,   -0.0814,
         0,   -0.0004,    0.9996,         0,   -0.4272,   -0.4272,
    0.0020,    0.0000,   -0.0000,    1.0000,    0.0000,   -0.0000,
         0,    0.0020,   -0.0000,         0,    0.9999,   -0.0001,
         0,   -0.0000,    0.0020,         0,   -0.0004,    0.9996;
        leg_lqr_param_.B_ <<  
    -0.0037,   -0.0056,
        0.0306,   -0.0687,
    -0.0452,   -0.3606,
    -0.0000,   -0.0000,
        0.0000,   -0.0001,
    -0.0000,   -0.0004;
        leg_lqr_param_.Q_ <<   1,              0,          0,          0,          0,               0,
                0,              1,          0,          0,          0,               0,
                0,              0,          1,          0,          0,               0,
                0,              0,          0,          1,          0,               0,
                0,              0,          0,          0,          1,               0,
                0,              0,          0,          0,          0,               1; 
        leg_lqr_param_.R_ <<   1,    0,
                0,      1; 

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
        state_interfaces_config.names.push_back(joint + "/" + "wx");
        state_interfaces_config.names.push_back(joint + "/" + "wy");
        state_interfaces_config.names.push_back(joint + "/" + "wz");
        state_interfaces_config.names.push_back(joint + "/" + "ax");
        state_interfaces_config.names.push_back(joint + "/" + "ay");
        state_interfaces_config.names.push_back(joint + "/" + "az");
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
    left_lqr_ = std::make_shared<LQR>();
    right_lqr_ = std::make_shared<LQR>();
    lqr_ = std::make_shared<LQR>();
    /// real leg data L1 L2 L3 L4 L5
    left_five_bar_ = std::make_shared<five_bar_linkage::FiveBar>(0.136,0.2774,0.2774,0.136,0.200);
    left_vmc_ = std::make_shared<VMC>();
    left_Fy_pid_ = std::make_shared<MiniPID>(0,0,0);

    right_five_bar_ = std::make_shared<five_bar_linkage::FiveBar>(0.1478,0.285,0.285,0.1478,0.18);
    right_vmc_ = std::make_shared<VMC>();
    right_Fy_pid_ = std::make_shared<MiniPID>(0,0,0);
   
    try {
        auto node = get_node();
        auto_declare("joint1_name", "");
        auto_declare("joint2_name", "");
        auto_declare("joint3_name", "");
        auto_declare("joint_Go_LF_name", "");
        auto_declare("joint_Go_LB_name", "");
        auto_declare("joint_Go_RF_name", "");
        auto_declare("joint_Go_RB_name", "");

        auto_declare("body_mg",0.0);

        auto_declare("fy_left_pid_p",0.0);
        auto_declare("fy_left_pid_i",0.0);
        auto_declare("fy_left_pid_d",0.0);

        auto_declare("fy_right_pid_p",0.0);
        auto_declare("fy_right_pid_i",0.0);
        auto_declare("fy_right_pid_d",0.0);

        auto_declare("left_x_q",0.0);
        auto_declare("left_x_dot_q",0.0);
        auto_declare("left_theta_q",0.0);
        auto_declare("left_theta_dot_q",0.0);
        auto_declare("left_fai_q",0.0);
        auto_declare("left_fai_dot_q",0.0);

        auto_declare("left_r1",0.0);
        auto_declare("left_r2",0.0);

        auto_declare("right_x_q",0.0);
        auto_declare("right_x_dot_q",0.0);
        auto_declare("right_theta_q",0.0);
        auto_declare("right_theta_dot_q",0.0);
        auto_declare("right_fai_q",0.0);
        auto_declare("right_fai_dot_q",0.0);

        auto_declare("right_r1",0.0);
        auto_declare("right_r2",0.0);
    }
    catch (const std::exception & e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
}

controller_interface::return_type WheelBalancingController::update()
{
/////////////////////// data prehandle ///////////////////////////
//update the remote date.
WheelBalancingController::updatingRemoteData();
// get the feedback 
WheelBalancingController::updateDataFromInterface();

/***************lqr*******************/
///   calculate the lqr k.
//    lqr_->K = lqr_->calcGainK();
//     lqr_->K  <<
//     4.3741,    1.4480,   -0.3754,    0.0972,   10.3204,   -2.1473,
//    -0.5014,   -0.1122,   -0.2845,   -0.0112,    0.3149,   -0.2761;
//     /// set the x_d to struct
//     WheelBalancingController::updateXdes();
//     /// give k to controller
//     WheelBalancingController::setLegLQRGain(lqr_->K);
//     /// set the x_d to controller
//     WheelBalancingController::setLegLQRXd();
//     /// set the X to controller
//     WheelBalancingController::setLegLQRX();
//     /// calculate the input of system. tau_W tau_Tp
//     WheelBalancingController::calclegLQRU();
//     temp_target_length_ = 0.3f;
//     double Tp = 0.0;
//     Tp = balance_controller_.U(1,0);   //adjust motor rotation dirextion.
/***************lqr*******************/

/***************vmc*******************/
    /// use these data
    /// give the L0 , theata and five bar linkage to vmc
    left_vmc_->getDataOfLeg(left_five_bar_->exportBarLength(),left_five_bar_->exportLinkageParam());
    /// calc the Fy /// give the Fy to vmc
    double Fy_left_output = 0.0f;
    Fy_left_output = left_Fy_pid_->getOutput(left_five_bar_->exportBarLength()->L0,temp_target_length_) ;
    Fy_left_output = -Fy_left_output + body_mg_;
    left_vmc_->setFTp(0.0f,Fy_left_output);
    /// calc the vmc output jacobian matrix
    left_vmc_->VMCControllerCalc();
    /// calc the vmc output 
    left_vmc_->calcT();
/***************vmc*******************/

    #ifdef NO_SIMULATION  
    /// give to send data
    send_data_.left_T1 = left_vmc_->exportT1() /  G01_REDUCTION_RATIO;  //adjust motor rotation dirextion.
    send_data_.left_T2 = left_vmc_->exportT2() / G01_REDUCTION_RATIO;  //adjust motor rotation dirextion.

    // send_data_.right_T1 = right_vmc_->exportT1() / G01_REDUCTION_RATIO;
    // send_data_.right_T2 = right_vmc_->exportT2() / G01_REDUCTION_RATIO;
    #else
    send_data_.left_T1 = left_vmc_->exportT1();  //adjust motor rotation dirextion.
    send_data_.left_T2 = -left_vmc_->exportT2();  //adjust motor rotation dirextion.

    send_data_.right_T1 = right_vmc_->exportT1();
    send_data_.right_T2 = right_vmc_->exportT2();
    // std::cout << "T1:" << send_data_.left_T1 << std::endl;
    // std::cout << "T2" <<  send_data_.left_T2 << std::endl;
    #endif

/***************transmit*******************/
    if(rc_commmonds_.sw_l == 1 || rc_commmonds_.sw_l == 0) { //protection mode
        // LK_L_handles_->set_torque(0.0f); // >0 forward
        // LK_R_handles_->set_torque(0.0f);
        // /// right leg //    - + up       + - down
        // Go1_RF_handles_->set_torque(0.0f);
        // Go1_RB_handles_->set_torque(0.0f);
        /// left leg //     + - down       - + up
        Go1_LF_handles_->set_torque(0.0f);
        Go1_LB_handles_->set_torque(0.0f);
    } else {//other modes
        LK_L_handles_->set_torque(0.0f); // >0 forward
        LK_R_handles_->set_torque(0.0f);
        // // // /// right leg //    - + up       + - down
        // Go1_LF_handles_->set_torque(0.0f);
        // Go1_LB_handles_->set_torque(-0.0f);
        // // // /// left leg
        // Go1_RF_handles_->set_torque(0.0f);
        // Go1_RB_handles_->set_torque(0.0f);
        /// left leg
        Go1_LB_handles_->set_torque(send_data_.left_T1);    //fai1
        Go1_LF_handles_->set_torque(send_data_.left_T2);    //fai4
        /// right leg
        // Go1_RF_handles_->set_torque(send_data_.right_T1);
        // Go1_RB_handles_->set_torque(send_data_.right_T2);
    }
/***************transmit*******************/

///////////////////// publish topic ////////////////////////////
#ifdef LQR_DEBUG
    if (realtime_LQR_debug_data_publisher_->trylock())
    {
     auto & lqr_debug_data_message = realtime_LQR_debug_data_publisher_->msg_;
     // left
     lqr_debug_data_message.x_now =state_var_now_.x;
     lqr_debug_data_message.x_dot_now= state_var_now_.x_dot;
     lqr_debug_data_message.theta_now= state_var_now_.theta_now;
     lqr_debug_data_message.theta_dot_now= state_var_now_.theta_dot;
     lqr_debug_data_message.fai_now= state_var_now_.fai;
     lqr_debug_data_message.fai_dot_now= state_var_now_.fai_dot;
     lqr_debug_data_message.x_des= 0.0f;
     lqr_debug_data_message.x_dot_des= 0.0f;
     lqr_debug_data_message.theta_des= 0.0f;
     lqr_debug_data_message.theta_dot_des= 0.0f;
     lqr_debug_data_message.fai_des= 0.0f;
     lqr_debug_data_message.fai_dot_des= 0.0f;
     lqr_debug_data_message.tauw_out = balance_controller_.U(0,0);
     lqr_debug_data_message.tp_out =balance_controller_.U(1,0);
     lqr_debug_data_message.t1_out = send_data_.left_T1;
     lqr_debug_data_message.t2_out = send_data_.left_T2;
     realtime_LQR_debug_data_publisher_->unlockAndPublish();
    }
#endif

#ifdef VMC_DEBUG
    if (realtime_VMC_debug_data_publisher_->trylock())
    {
        auto & vmc_debug_data_message = realtime_VMC_debug_data_publisher_->msg_;
        vmc_debug_data_message.left_t_one = send_data_.left_T1;
        vmc_debug_data_message.left_t_two = send_data_.left_T2;
        vmc_debug_data_message.left_tp = 0.0f;
        vmc_debug_data_message.left_f = Fy_left_output;
        vmc_debug_data_message.left_fai_one = need_data_form_hi_.left_leg_fai1;
        vmc_debug_data_message.left_fai_four = need_data_form_hi_.left_leg_fai4;
        vmc_debug_data_message.left_target_l =temp_target_length_;
        vmc_debug_data_message.left_actual_l = left_five_bar_->exportBarLength()->L0;
        vmc_debug_data_message.left_pid_fy = Fy_left_output;
        vmc_debug_data_message.left_p_out = 0.0f;
        vmc_debug_data_message.left_i_out = 0.0f;
        vmc_debug_data_message.left_d_out = 0.0f;

        vmc_debug_data_message.right_t_one = send_data_.right_T1;
        vmc_debug_data_message.right_t_two = send_data_.right_T2;
        vmc_debug_data_message.right_tp = 0.0f;
        vmc_debug_data_message.right_f = 0.0f;
        vmc_debug_data_message.right_fai_one = need_data_form_hi_.right_leg_fai1;
        vmc_debug_data_message.right_fai_four = need_data_form_hi_.right_leg_fai4;
        vmc_debug_data_message.right_target_l = rc_commmonds_.ch_r_y;
        vmc_debug_data_message.right_actual_l = right_five_bar_->exportBarLength()->L0;
        vmc_debug_data_message.right_pid_fy = 0.0f;
        vmc_debug_data_message.right_p_out = 0.0f;
        vmc_debug_data_message.right_i_out = 0.0f;
        vmc_debug_data_message.right_d_out = 0.0f;
        realtime_VMC_debug_data_publisher_->unlockAndPublish();
    }
#endif

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
    #ifdef NO_SIMULATION
        if (realtime_Go1_data_publisher_->trylock())
        {
        auto & Go1_data_message = realtime_Go1_data_publisher_->msg_;
        Go1_data_message.lfpositions = Go1_LF_handles_->get_position() / G01_REDUCTION_RATIO * (180.0f/PI);
        Go1_data_message.lfvelocities = Go1_LF_handles_->get_velocity() * G01_REDUCTION_RATIO;
        Go1_data_message.lftorques = Go1_LF_handles_->get_acceleration();

        Go1_data_message.rfpositions = Go1_RF_handles_->get_position() / G01_REDUCTION_RATIO *(180.0f/PI);
        Go1_data_message.rfvelocities = Go1_RF_handles_->get_velocity();
        Go1_data_message.rftorques = Go1_RF_handles_->get_acceleration() * G01_REDUCTION_RATIO;

        Go1_data_message.rbpositions = Go1_RB_handles_->get_position() / G01_REDUCTION_RATIO* (180.0f/PI);
        Go1_data_message.rbvelocities = Go1_RB_handles_->get_velocity();
        Go1_data_message.rbtorques = Go1_RB_handles_->get_acceleration() * G01_REDUCTION_RATIO;
        
        Go1_data_message.lbpositions = Go1_LB_handles_->get_position() / G01_REDUCTION_RATIO*(180.0f/PI);
        Go1_data_message.lbvelocities = Go1_LB_handles_->get_velocity();
        Go1_data_message.lbtorques = Go1_LB_handles_->get_acceleration() * G01_REDUCTION_RATIO;
        realtime_Go1_data_publisher_->unlockAndPublish();
        }
    #else
        if (realtime_Go1_data_publisher_->trylock())
        {
        auto & Go1_data_message = realtime_Go1_data_publisher_->msg_;
        Go1_data_message.lfpositions = need_data_form_hi_.lf_go1_pos;
        Go1_data_message.lfvelocities =need_data_form_hi_.lf_go1_vel;
        Go1_data_message.lftorques = 0.0f;

        Go1_data_message.rfpositions = need_data_form_hi_.rf_go1_pos;
        Go1_data_message.rfvelocities = need_data_form_hi_.rf_go1_vel;
        Go1_data_message.rftorques = 0.0f;

        Go1_data_message.rbpositions = need_data_form_hi_.rb_go1_pos;
        Go1_data_message.rbvelocities = need_data_form_hi_.rb_go1_vel;
        Go1_data_message.rbtorques = 0.0f;
        
        Go1_data_message.lbpositions = need_data_form_hi_.lb_go1_pos;
        Go1_data_message.lbvelocities = need_data_form_hi_.lb_go1_vel;
        Go1_data_message.lbtorques = 0.0f;
        realtime_Go1_data_publisher_->unlockAndPublish();
        }        
    #endif
#endif

#ifdef SIMULATION
if (realtime_Simulation_data_publisher_->trylock())
{
    auto & simulation_data_message = realtime_Simulation_data_publisher_->msg_;
    simulation_data_message.torque_lb = 0.00f;
    simulation_data_message.torque_lf = 0.00f;
    simulation_data_message.torque_rb = 0.00f;
    simulation_data_message.torque_rf = 0.00f;

    simulation_data_message.torque_wl = 0.00f;
    simulation_data_message.torque_wr = 0.00f;
    realtime_Simulation_data_publisher_->unlockAndPublish();
}
if (realtime_torque_lb_publisher_->trylock())
{
    auto & torque_lb_message = realtime_torque_lb_publisher_->msg_;
    torque_lb_message.data = send_data_.left_T2;
    realtime_torque_lb_publisher_->unlockAndPublish();
}

if (realtime_torque_lf_publisher_->trylock())
{
    auto & torque_lf_message = realtime_torque_lf_publisher_->msg_;
    torque_lf_message.data = send_data_.left_T1;
    realtime_torque_lf_publisher_->unlockAndPublish();
}
// if (realtime_torque_rb_publisher_->trylock())
// {
//     auto & torque_rb_message = realtime_torque_rb_publisher_->msg_;
//     torque_rb_message.data = send_data_.right_T2;
//     realtime_torque_rb_publisher_->unlockAndPublish();
// }
// if (realtime_torque_rf_publisher_->trylock())
// {
//     auto & torque_rf_message = realtime_torque_rf_publisher_->msg_;
//     torque_rf_message.data = send_data_.right_T1;
//     realtime_torque_rf_publisher_->unlockAndPublish();
// }
// if (realtime_torque_wl_publisher_->trylock())
// {
//     auto & torque_wl_message = realtime_torque_wl_publisher_->msg_;
//     torque_wl_message.data = send_data_.left_tau_w;
//     realtime_torque_wl_publisher_->unlockAndPublish();
// }
// if (realtime_torque_wr_publisher_->trylock())
// {
//     auto & torque_wr_message = realtime_torque_wr_publisher_->msg_;
//     torque_wr_message.data = send_data_.right_tau_w;;
//     realtime_torque_wr_publisher_->unlockAndPublish();
// }
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

    get_node()->get_parameter("body_mg",body_mg_);
    
    get_node()->get_parameter("fy_left_pid_p",left_Fy_pid_->P);
    get_node()->get_parameter("fy_left_pid_i",left_Fy_pid_->I);
    get_node()->get_parameter("fy_left_pid_d",left_Fy_pid_->D);

    get_node()->get_parameter("fy_right_pid_p",right_Fy_pid_->P);
    get_node()->get_parameter("fy_right_pid_i",right_Fy_pid_->I);
    get_node()->get_parameter("fy_right_pid_d",right_Fy_pid_->D);
    
    get_node()->get_parameter("x_dot_q",leg_lqr_param_.Q_(0,0));
    get_node()->get_parameter("theta_dot_q",leg_lqr_param_.Q_(1,1));
    get_node()->get_parameter("fai_dot_q",leg_lqr_param_.Q_(2,2));
    get_node()->get_parameter("x_q",leg_lqr_param_.Q_(3,3));
    get_node()->get_parameter("theta_q",leg_lqr_param_.Q_(4,4));
    get_node()->get_parameter("fai_q",leg_lqr_param_.Q_(5,5));
    

    get_node()->get_parameter("r1",leg_lqr_param_.R_(0,0));
    get_node()->get_parameter("r2",leg_lqr_param_.R_(1,1));

    WheelBalancingController::initLQRParam();

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

    simu_imu_subsciption_ = get_node()->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, [this](const std::shared_ptr<sensor_msgs::msg::Imu> simu_imu_fdb)
    {
        simu_imu_ptr_.writeFromNonRT(simu_imu_fdb);
    });
    simu_leg_lb_p_subsciption_ = get_node()->create_subscription<std_msgs::msg::Float64>("/leg_lb_p", 10, [this](const std::shared_ptr<std_msgs::msg::Float64> simu_leg_lb_p_fdb)
    {
        simu_leg_lb_p_ptr_.writeFromNonRT(simu_leg_lb_p_fdb);
    });
        simu_leg_lb_v_subsciption_ = get_node()->create_subscription<std_msgs::msg::Float64>("/leg_lb_v", 10, [this](const std::shared_ptr<std_msgs::msg::Float64> simu_leg_lb_v_fdb)
    {
        simu_leg_lb_v_ptr_.writeFromNonRT(simu_leg_lb_v_fdb);
    });
        simu_leg_lf_p_subsciption_ = get_node()->create_subscription<std_msgs::msg::Float64>("/leg_lf_p", 10, [this](const std::shared_ptr<std_msgs::msg::Float64> simu_leg_lf_p_fdb)
    {
        simu_leg_lf_p_ptr_.writeFromNonRT(simu_leg_lf_p_fdb);
    });
        simu_leg_lf_v_subsciption_ = get_node()->create_subscription<std_msgs::msg::Float64>("/leg_lf_v", 10, [this](const std::shared_ptr<std_msgs::msg::Float64> simu_leg_lf_v_fdb)
    {
        simu_leg_lf_v_ptr_.writeFromNonRT(simu_leg_lf_v_fdb);
    });
        simu_leg_rb_p_subsciption_ = get_node()->create_subscription<std_msgs::msg::Float64>("/leg_rb_p", 10, [this](const std::shared_ptr<std_msgs::msg::Float64> simu_leg_rb_p_fdb)
    {
        simu_leg_rb_p_ptr_.writeFromNonRT(simu_leg_rb_p_fdb);
    });
        simu_leg_rb_v_subsciption_ = get_node()->create_subscription<std_msgs::msg::Float64>("/leg_rb_v", 10, [this](const std::shared_ptr<std_msgs::msg::Float64> simu_leg_rb_v_fdb)
    {
        simu_leg_rb_v_ptr_.writeFromNonRT(simu_leg_rb_v_fdb);
    });
        simu_leg_rf_p_subsciption_ = get_node()->create_subscription<std_msgs::msg::Float64>("/leg_rf_p", 10, [this](const std::shared_ptr<std_msgs::msg::Float64> simu_leg_rf_p_fdb)
    {
        simu_leg_rf_p_ptr_.writeFromNonRT(simu_leg_rf_p_fdb);
    });
        simu_leg_rf_v_subsciption_ = get_node()->create_subscription<std_msgs::msg::Float64>("/leg_rf_v", 10, [this](const std::shared_ptr<std_msgs::msg::Float64> simu_leg_rf_v_fdb)
    {
        simu_leg_rf_v_ptr_.writeFromNonRT(simu_leg_rf_v_fdb);
    });
        simu_wheelleft_p_subsciption_ = get_node()->create_subscription<std_msgs::msg::Float64>("/wheelleft_p", 10, [this](const std::shared_ptr<std_msgs::msg::Float64> simu_wheelleft_p_fdb)
    {
        simu_wheelleft_p_ptr_.writeFromNonRT(simu_wheelleft_p_fdb);
    });
        simu_wheelleft_v_subsciption_ = get_node()->create_subscription<std_msgs::msg::Float64>("/wheelleft_v", 10, [this](const std::shared_ptr<std_msgs::msg::Float64> simu_wheelleft_v_fdb)
    {
        simu_wheelleft_v_ptr_.writeFromNonRT(simu_wheelleft_v_fdb);
    });
        simu_wheelright_p_subsciption_ = get_node()->create_subscription<std_msgs::msg::Float64>("/wheelright_p", 10, [this](const std::shared_ptr<std_msgs::msg::Float64> simu_wheelright_p_fdb)
    {
        simu_wheelright_p_ptr_.writeFromNonRT(simu_wheelright_p_fdb);
    });
        simu_wheelright_v_subsciption_ = get_node()->create_subscription<std_msgs::msg::Float64>("/wheelright_v", 10, [this](const std::shared_ptr<std_msgs::msg::Float64> simu_wheelright_v_fdb)
    {
        simu_wheelright_v_ptr_.writeFromNonRT(simu_wheelright_v_fdb);
    });
#ifdef LQR_DEBUG
    LQR_debug_data_publisher_ =  get_node()->create_publisher<warrior_interface::msg::LQRDebugData>("/lqr_debug_feedback", 
        rclcpp::SystemDefaultsQoS());
    realtime_LQR_debug_data_publisher_ = 
    std::make_shared<realtime_tools::RealtimePublisher<warrior_interface::msg::LQRDebugData>>(
        LQR_debug_data_publisher_);
     auto & lqr_debug_data_message = realtime_LQR_debug_data_publisher_->msg_;
     // left
     
     lqr_debug_data_message.x_now = 0.0f;
     lqr_debug_data_message.x_dot_now= 0.0f;
     lqr_debug_data_message.theta_now= 0.0f;
     lqr_debug_data_message.theta_dot_now= 0.0f;
     lqr_debug_data_message.fai_now= 0.0f;
     lqr_debug_data_message.fai_dot_now= 0.0f;
     lqr_debug_data_message.x_des= 0.0f;
     lqr_debug_data_message.x_dot_des= 0.0f;
     lqr_debug_data_message.theta_des= 0.0f;
     lqr_debug_data_message.theta_dot_des= 0.0f;
     lqr_debug_data_message.fai_des= 0.0f;
     lqr_debug_data_message.fai_dot_des= 0.0f;
     lqr_debug_data_message.tauw_out= 0.0f;
     lqr_debug_data_message.tp_out= 0.0f;
     lqr_debug_data_message.t1_out= 0.0f;
     lqr_debug_data_message.t2_out= 0.0f;
#endif

#ifdef VMC_DEBUG
    VMC_debug_data_publisher_ = get_node()->create_publisher<warrior_interface::msg::VMCDebugData>("/vmc_debug_feedback", 
        rclcpp::SystemDefaultsQoS());
    realtime_VMC_debug_data_publisher_ = 
        std::make_shared<realtime_tools::RealtimePublisher<warrior_interface::msg::VMCDebugData>>(
            VMC_debug_data_publisher_);
    auto & vmc_debug_data_message = realtime_VMC_debug_data_publisher_->msg_;
    vmc_debug_data_message.left_t_one = 0.0f;
    vmc_debug_data_message.left_t_two = 0.0f;
    vmc_debug_data_message.left_tp = 0.0f;
    vmc_debug_data_message.left_f = 0.0f;
    vmc_debug_data_message.left_fai_one = 0.0f;
    vmc_debug_data_message.left_fai_four = 0.0f;
    vmc_debug_data_message.left_target_l = 0.0f;
    vmc_debug_data_message.left_actual_l = 0.0f;
    vmc_debug_data_message.left_pid_fy = 0.0f;
    vmc_debug_data_message.left_p_out = 0.0f;
    vmc_debug_data_message.left_i_out = 0.0f;
    vmc_debug_data_message.left_d_out = 0.0f;

    vmc_debug_data_message.right_t_one = 0.0f;
    vmc_debug_data_message.right_t_two = 0.0f;
    vmc_debug_data_message.right_tp = 0.0f;
    vmc_debug_data_message.right_f = 0.0f;
    vmc_debug_data_message.right_fai_one = 0.0f;
    vmc_debug_data_message.right_fai_four = 0.0f;
    vmc_debug_data_message.right_target_l = 0.0f;
    vmc_debug_data_message.right_actual_l = 0.0f;
    vmc_debug_data_message.right_pid_fy = 0.0f;
    vmc_debug_data_message.right_p_out = 0.0f;
    vmc_debug_data_message.right_i_out = 0.0f;
    vmc_debug_data_message.right_d_out = 0.0f;
#endif
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
    Go1_data_message.lfpositions = 0.0f;
    Go1_data_message.lfvelocities =0.0f;
    Go1_data_message.lftorques = 0.00;

    Go1_data_message.rfpositions = 0.0f;
    Go1_data_message.rfvelocities = 0.0f;
    Go1_data_message.rftorques = 0.00;

    Go1_data_message.rbpositions = 0.0f;
    Go1_data_message.rbvelocities = 0.0f;
    Go1_data_message.rbtorques = 0.00;

    Go1_data_message.lbpositions = 0.0f;
    Go1_data_message.lbvelocities = 0.0f;
    Go1_data_message.lbtorques = 0.00;
#endif

#ifdef SIMULATION
    Simulation_data_publisher_ = get_node()->create_publisher<warrior_interface::msg::SimulationData>("/simulation_data", 
        rclcpp::SystemDefaultsQoS());
    realtime_Simulation_data_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<warrior_interface::msg::SimulationData>>(
            Simulation_data_publisher_);
    auto & simulation_data_message = realtime_Simulation_data_publisher_->msg_;
    simulation_data_message.torque_lb = 0.00;
    simulation_data_message.torque_lf = 0.00;
    simulation_data_message.torque_rb = 0.00;

    simulation_data_message.torque_rf = 0.00;
    simulation_data_message.torque_wl = 0.00;
    simulation_data_message.torque_wr = 0.00;

    torque_lb_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>("/torque_lb", 
        rclcpp::SystemDefaultsQoS());
    realtime_torque_lb_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(
        torque_lb_publisher_);
    auto & torque_lb_message = realtime_torque_lb_publisher_->msg_;
    torque_lb_message.data = 0.0f;

        torque_lf_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>("/torque_lf", 
        rclcpp::SystemDefaultsQoS());
    realtime_torque_lf_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(
        torque_lf_publisher_);
    auto & torque_lf_message = realtime_torque_lf_publisher_->msg_;
    torque_lf_message.data = 0.0f;

        torque_rb_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>("/torque_rb", 
        rclcpp::SystemDefaultsQoS());
    realtime_torque_rb_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(
        torque_rb_publisher_);
    auto & torque_rb_message = realtime_torque_rb_publisher_->msg_;
    torque_rb_message.data = 0.0f;

        torque_rf_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>("/torque_rf", 
        rclcpp::SystemDefaultsQoS());
    realtime_torque_rf_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(
        torque_rf_publisher_);
    auto & torque_rf_message = realtime_torque_rf_publisher_->msg_;
    torque_rf_message.data = 0.0f;

        torque_wl_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>("/torque_wl", 
        rclcpp::SystemDefaultsQoS());
    realtime_torque_wl_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(
        torque_wl_publisher_);
    auto & torque_wl_message = realtime_torque_wl_publisher_->msg_;
    torque_wl_message.data = 0.0f;

        torque_wr_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>("/torque_wr", 
        rclcpp::SystemDefaultsQoS());
    realtime_torque_wr_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(
        torque_wr_publisher_);
    auto & torque_wr_message = realtime_torque_wr_publisher_->msg_;
    torque_wr_message.data = 0.0f;

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
    // WheelBalancingController::InitXdes(0);  

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
     rc_commmonds_.ch_l_x  = rc->ch_l_x   * 999;
     rc_commmonds_.ch_l_y  = rc->ch_l_y   * 999;
     rc_commmonds_.ch_r_x  = rc->ch_r_x   * 999;
     /* L_-1 */
     rc_commmonds_.ch_r_y  += (rc->ch_r_y * -1.001 * 0.001f);
     if(rc_commmonds_.ch_r_y<MIN_L0)rc_commmonds_.ch_r_y = MIN_L0;
     if(rc_commmonds_.ch_r_y>MAX_L0)rc_commmonds_.ch_r_y = MAX_L0;
     
    //  std::cout << "ch_y: " << "      " << rc_commmonds_.ch_r_y << std::endl;
     rc_commmonds_.sw_l = rc->s_l;
     rc_commmonds_.sw_r = rc->s_r;
     rc_commmonds_.wheel = rc->wheel;
    return controller_interface::return_type::OK;
}

controller_interface::return_type WheelBalancingController::updatingSimuImuData(void)
{
    // look the subcription
    auto command = simu_imu_ptr_.readFromRT();
    if (!command || !(*command)) {
        return controller_interface::return_type::OK;
    }
    const auto simu_imu_data = (*command);
    double x(0.0),y(0.0),z(0.0),w(0.0);
    x = simu_imu_data->orientation.x;
    y = simu_imu_data->orientation.y;
    z = simu_imu_data->orientation.z;
    w = simu_imu_data->orientation.w;
    need_data_form_hi_.yaw   = atan2f(y*x+z*w,
                                        y*y+z*z-0.5f);

    need_data_form_hi_.pitch = asinf(2*(y*w-z*x));

    need_data_form_hi_.roll  = atan2f(y*z+w*x,
                                        y*y+x*x-0.5f);
    need_data_form_hi_.wx = simu_imu_data->angular_velocity.x;
    need_data_form_hi_.wy = simu_imu_data->angular_velocity.y;
    need_data_form_hi_.wz = simu_imu_data->angular_velocity.z;

    need_data_form_hi_.ax = simu_imu_data->linear_acceleration.x;
    need_data_form_hi_.ay = simu_imu_data->linear_acceleration.y;
    need_data_form_hi_.az = simu_imu_data->linear_acceleration.z;
    // std::cout << "need_data_form_hi_.pitch " << need_data_form_hi_.pitch  << std::endl;
    return controller_interface::return_type::OK;
}

controller_interface::return_type WheelBalancingController::updatingSimuMotorData(void)
{
    // look the subcription
    auto command = simu_leg_lb_p_ptr_.readFromRT();
    if (!command || !(*command)) {
        return controller_interface::return_type::OK;
    }
    auto motor_data = (*command);
    need_data_form_hi_.lb_go1_pos = motor_data->data;
    // std::cout << "lb_go1_pos" << need_data_form_hi_.lb_go1_pos << std::endl;
    // look the subcription
    command = simu_leg_lb_v_ptr_.readFromRT();
    if (!command || !(*command)) {
        return controller_interface::return_type::OK;
    } 
    motor_data = (*command);
    need_data_form_hi_.lb_go1_vel = motor_data->data;
    // std::cout << "lb_go1_vel" << need_data_form_hi_.lb_go1_vel << std::endl;

    // look the subcription
    command = simu_leg_lf_p_ptr_.readFromRT();
    if (!command || !(*command)) {
        return controller_interface::return_type::OK;
    } 
    motor_data = (*command);
    need_data_form_hi_.lf_go1_pos = motor_data->data;
    // std::cout << "lf_go1_pos" << need_data_form_hi_.lf_go1_pos << std::endl;
    
    // look the subcription
    command = simu_leg_lf_v_ptr_.readFromRT();
    if (!command || !(*command)) {
        return controller_interface::return_type::OK;
    } 
    motor_data = (*command);
    need_data_form_hi_.lf_go1_vel = motor_data->data;
    // std::cout << "lf_go1_vel" << need_data_form_hi_.lf_go1_vel << std::endl;

    // look the subcription
    command = simu_leg_rb_p_ptr_.readFromRT();
    if (!command || !(*command)) {
        return controller_interface::return_type::OK;
    } 
    motor_data = (*command);
    need_data_form_hi_.rb_go1_pos = motor_data->data;
    // std::cout << "rb_go1_pos   " << need_data_form_hi_.rb_go1_pos << std::endl;

    // look the subcription
    command = simu_leg_rb_v_ptr_.readFromRT();
    if (!command || !(*command)) {
        return controller_interface::return_type::OK;
    } 
    motor_data = (*command);
    need_data_form_hi_.rb_go1_vel = motor_data->data;
    // std::cout << "rb_go1_vel   " << need_data_form_hi_.rb_go1_vel << std::endl;

    // look the subcription
    command = simu_leg_rf_p_ptr_.readFromRT();
    if (!command || !(*command)) {
        return controller_interface::return_type::OK;
    } 
    motor_data = (*command);
    need_data_form_hi_.rf_go1_pos = motor_data->data;
    // std::cout << "rf_go1_pos  " << need_data_form_hi_.rf_go1_pos << std::endl;
    
    // look the subcription
    command = simu_leg_rf_v_ptr_.readFromRT();
    if (!command || !(*command)) {
        return controller_interface::return_type::OK;
    } 
    motor_data = (*command);
    need_data_form_hi_.rf_go1_vel = motor_data->data;
    // std::cout << "rf_go1_vel  " << need_data_form_hi_.rf_go1_vel << std::endl;

    // look the subcription
    command = simu_wheelleft_p_ptr_.readFromRT();
    if (!command || !(*command)) {
        return controller_interface::return_type::OK;
    } 
    motor_data = (*command);
    need_data_form_hi_.left_lk9025_pos = motor_data->data;
    // std::cout << "left_lk9025_pos  " << need_data_form_hi_.left_lk9025_pos << std::endl;

    // look the subcription
    command = simu_wheelleft_v_ptr_.readFromRT();
    if (!command || !(*command)) {
        return controller_interface::return_type::OK;
    } 
    motor_data = (*command);
    need_data_form_hi_.left_lk9025_vel = motor_data->data;
    // std::cout << "left_lk9025_vel  " << need_data_form_hi_.left_lk9025_vel << std::endl;

    // look the subcription
    command = simu_wheelright_p_ptr_.readFromRT();
    if (!command || !(*command)) {
        return controller_interface::return_type::OK;
    } 
    motor_data = (*command);
    need_data_form_hi_.right_lk9025_pos = motor_data->data;
    // std::cout << "right_lk9025_pos  " << need_data_form_hi_.right_lk9025_pos << std::endl;
    
    // look the subcription
    command = simu_wheelright_v_ptr_.readFromRT();
    if (!command || !(*command)) {
        return controller_interface::return_type::OK;
    } 
    motor_data = (*command);
    need_data_form_hi_.right_lk9025_vel = motor_data->data;
    // std::cout << "right_lk9025_vel  " << need_data_form_hi_.right_lk9025_vel << std::endl;

    return controller_interface::return_type::OK;    
}
// 
void WheelBalancingController::updateDataFromInterface(void)
{
#ifdef SIMULATION
    /// data pre-handle
    WheelBalancingController::init9025EncoderZeros();
    // motor
    WheelBalancingController::updatingSimuMotorData();
    need_data_form_hi_.left_leg_dis_dot =  ((need_data_form_hi_.left_lk9025_vel * (PI /180.0f)) * DRIVER_RADIUS);
    need_data_form_hi_.right_leg_dis_dot = ((need_data_form_hi_.right_lk9025_vel * (PI /180.0f)) * DRIVER_RADIUS);
    // imu
    WheelBalancingController::updatingSimuImuData();
    
    /// left leg param    
    need_data_form_hi_.left_leg_fai4 = need_data_form_hi_.lf_go1_pos;
    need_data_form_hi_.left_leg_fai1 = PI - need_data_form_hi_.lb_go1_pos;
    /// right leg param
    need_data_form_hi_.right_leg_fai4 = PI - (RIGHT_LEG_FAI_ZERO - (need_data_form_hi_.rb_go1_pos - GO1_2_ZEROS));
    need_data_form_hi_.right_leg_fai1 = (RIGHT_LEG_FAI_ZERO - (need_data_form_hi_.rf_go1_pos  - GO1_1_ZEROS));

    WheelBalancingController::update9025TotalDis();
    /// get x  
    WheelBalancingController::updateX();
    /// get theta
    left_five_bar_->virtualLegCalc(need_data_form_hi_.left_leg_fai1, need_data_form_hi_.left_leg_fai4);
    right_five_bar_->virtualLegCalc(need_data_form_hi_.right_leg_fai1, need_data_form_hi_.right_leg_fai4);
#endif

#ifdef NO_SIMULATION
    /// data pre-handle
    WheelBalancingController::init9025EncoderZeros();

    need_data_form_hi_.left_lk9025_pos = LK_L_handles_->get_position();
    need_data_form_hi_.left_lk9025_vel = LK_L_handles_->get_velocity();
    need_data_form_hi_.left_lk9025_tor = LK_L_handles_->get_acceleration();

    need_data_form_hi_.right_lk9025_pos = LK_R_handles_->get_position();
    need_data_form_hi_.right_lk9025_vel = LK_R_handles_->get_velocity();
    need_data_form_hi_.right_lk9025_tor = LK_R_handles_->get_acceleration();

    need_data_form_hi_.lf_go1_pos = Go1_LF_handles_->get_position();
    need_data_form_hi_.lf_go1_vel = Go1_LF_handles_->get_velocity();
    need_data_form_hi_.lf_go1_tor = Go1_LF_handles_->get_acceleration();

    need_data_form_hi_.rf_go1_pos = Go1_RB_handles_->get_position();
    need_data_form_hi_.rf_go1_vel = Go1_RB_handles_->get_velocity();
    need_data_form_hi_.rf_go1_tor = Go1_RB_handles_->get_acceleration();

    need_data_form_hi_.rb_go1_pos = Go1_RB_handles_->get_position();
    need_data_form_hi_.rb_go1_vel = Go1_RB_handles_->get_velocity();
    need_data_form_hi_.rb_go1_tor = Go1_RB_handles_->get_acceleration();

    need_data_form_hi_.lb_go1_pos = Go1_LB_handles_->get_position();
    need_data_form_hi_.lb_go1_vel = Go1_LB_handles_->get_velocity();
    need_data_form_hi_.lb_go1_tor = Go1_LB_handles_->get_acceleration();

    // need_data_form_hi_.left_leg_dis_dot = ((need_data_form_hi_.left_lk9025_vel * (PI /180.0f)) * DRIVER_RADIUS);
    need_data_form_hi_.left_leg_dis_dot =  ((need_data_form_hi_.left_lk9025_vel * (PI /180.0f)) * DRIVER_RADIUS);
    need_data_form_hi_.right_leg_dis_dot = ((need_data_form_hi_.right_lk9025_vel * (PI /180.0f)) * DRIVER_RADIUS);

    need_data_form_hi_.pitch = imu_handles_->get_pitch(); 
    need_data_form_hi_.yaw = imu_handles_->get_yaw();
    need_data_form_hi_.roll = imu_handles_->get_roll();

    need_data_form_hi_.wx = imu_handles_->get_wx();
    need_data_form_hi_.wy = imu_handles_->get_wy();
    need_data_form_hi_.wz = imu_handles_->get_wz();

    need_data_form_hi_.ax = imu_handles_->get_ax();
    need_data_form_hi_.ay = imu_handles_->get_ay();
    need_data_form_hi_.az = imu_handles_->get_az();
    /// left leg param    
    need_data_form_hi_.left_leg_fai4 = PI - (LEFT_LEG_FAI_ZERO - (need_data_form_hi_.lf_go1_pos  - GO1_0_ZEROS) / G01_REDUCTION_RATIO);
    need_data_form_hi_.left_leg_fai1 = (LEFT_LEG_FAI_ZERO + (need_data_form_hi_.lb_go1_pos  - GO1_3_ZEROS) / G01_REDUCTION_RATIO);
    // std::cout << "fai4      " << need_data_form_hi_.left_leg_fai4 << endl;
    // std::cout << "fai1      " << need_data_form_hi_.left_leg_fai1 << endl;
    /// right leg param
    need_data_form_hi_.right_leg_fai4 = PI - (RIGHT_LEG_FAI_ZERO - (need_data_form_hi_.rb_go1_pos - GO1_2_ZEROS) / G01_REDUCTION_RATIO);
    need_data_form_hi_.right_leg_fai1 = (RIGHT_LEG_FAI_ZERO - (need_data_form_hi_.rf_go1_pos  - GO1_1_ZEROS) / G01_REDUCTION_RATIO);
    /// get theta
    left_five_bar_->virtualLegCalc(need_data_form_hi_.left_leg_fai1, need_data_form_hi_.left_leg_fai4);
    right_five_bar_->virtualLegCalc(need_data_form_hi_.right_leg_fai1, need_data_form_hi_.right_leg_fai4);
    WheelBalancingController::update9025TotalDis();
    /// get x  
    WheelBalancingController::updateX(); 

#endif


}

/******************************************************************************************/
/// banlance controller
/// 0 left 1 right 
/// get the value of all i will need in the controller.
controller_interface::return_type WheelBalancingController::init9025EncoderZeros(void)
{
    //  std::cout << " need_data_form_hi_.right_lk9025_ecoder_zero : " << need_data_form_hi_.right_lk9025_ecoder_zero  << std::endl;
    #ifdef NO_SIMULATION
    if(need_data_form_hi_.left_init_flag==0)
    {
        need_data_form_hi_.left_lk9025_pos = LK_L_handles_->get_position();
        need_data_form_hi_.left_lk9025_ecoder_zero = need_data_form_hi_.left_lk9025_pos;
        need_data_form_hi_.left_leg_total_dis = 0.0f;
        need_data_form_hi_.left_init_flag = 1;//zero point initilized flag
    }
    if(need_data_form_hi_.right_init_flag==0)
    {
        need_data_form_hi_.right_lk9025_pos = LK_R_handles_->get_position();
        need_data_form_hi_.right_lk9025_ecoder_zero = need_data_form_hi_.right_lk9025_pos;
        need_data_form_hi_.right_leg_total_dis = 0.0f;
        need_data_form_hi_.right_init_flag = 1;
    }
    #else
    if(need_data_form_hi_.left_init_flag==0)
    {
        // look the subcription
        auto command = simu_wheelleft_p_ptr_.readFromRT();
        if (!command || !(*command)) {
            return controller_interface::return_type::OK;
        } 
        auto motor_data = (*command);
        need_data_form_hi_.left_lk9025_pos = motor_data->data;
        need_data_form_hi_.left_lk9025_ecoder_zero = need_data_form_hi_.left_lk9025_pos;
        need_data_form_hi_.left_leg_total_dis = 0.0f;
        need_data_form_hi_.left_init_flag = 1;//zero point initilized flag
    }
    if(need_data_form_hi_.right_init_flag==0)
    {
        // look the subcription
        auto command = simu_wheelright_p_ptr_.readFromRT();
        if (!command || !(*command)) {
            return controller_interface::return_type::OK;
        } 
        auto motor_data = (*command);
        need_data_form_hi_.right_lk9025_pos = motor_data->data;
        need_data_form_hi_.right_lk9025_ecoder_zero = need_data_form_hi_.right_lk9025_pos;
        need_data_form_hi_.right_leg_total_dis = 0.0f;
        need_data_form_hi_.right_init_flag = 1;
    }
    #endif
}

void WheelBalancingController::update9025TotalDis(void)
{
    #ifdef NO_SIMULATION
    int8_t l_dir(0),r_dir(0);
    /// get the rotate direction.1 + -1 -
    if(need_data_form_hi_.left_lk9025_vel > 0)  l_dir = 1; else if(need_data_form_hi_.left_lk9025_vel<0) l_dir = -1; else l_dir = 0;
    if(need_data_form_hi_.right_lk9025_vel > 0)  r_dir = 1; else if(need_data_form_hi_.right_lk9025_vel<0) r_dir = -1; else r_dir = 0;
    /// calculate the distance
        //left
    if(l_dir == 1)
    {
        /// get circle cnt
            //overflow judge
        if(need_data_form_hi_.left_lk9025_ecoder_last - need_data_form_hi_.left_lk9025_pos > 0x7FFF)//forward rotate down overflow last(65534)-now(1) > 
        {
            need_data_form_hi_.left_lk9025_circle_cnt++;
        }
    }
    else if(l_dir == -1)
    {
        if(need_data_form_hi_.left_lk9025_pos - need_data_form_hi_.left_lk9025_ecoder_last > 0x7FFF)//back rotate up overflow now(65535) - last(0) > 
        {
             need_data_form_hi_.left_lk9025_circle_cnt--;
        }
    }
    else
    {
        need_data_form_hi_.left_lk9025_circle_cnt+=0;
    }
    /// total dis = circle_cnt * 65535 + (pos-zero)
    if(l_dir != 0)
        need_data_form_hi_.left_leg_total_dis = (need_data_form_hi_.left_lk9025_circle_cnt 
                    + ((need_data_form_hi_.left_lk9025_pos - need_data_form_hi_.left_lk9025_ecoder_zero) / 65535.0f))
                        * 2 * PI * DRIVER_RADIUS;
        /// right
    if(r_dir == 1)
    {
        if(need_data_form_hi_.right_lk9025_ecoder_last - need_data_form_hi_.right_lk9025_pos > 0x7FFF)//forward rotate down overflow last(65534)-now(1) > 
        {
            need_data_form_hi_.right_lk9025_circle_cnt++;
        }
    }
    else if(r_dir == -1)
    {
        if(need_data_form_hi_.right_lk9025_pos - need_data_form_hi_.right_lk9025_ecoder_last > 0x7FFF)//back rotate up overflow now(65535) - last(0) > 
        {
            need_data_form_hi_.right_lk9025_circle_cnt--;
        }
    }
    else
    {
        need_data_form_hi_.right_lk9025_circle_cnt += 0;
    }
    if(r_dir != 0)
        need_data_form_hi_.right_leg_total_dis = (need_data_form_hi_.right_lk9025_circle_cnt 
                    + ( (need_data_form_hi_.right_lk9025_pos - need_data_form_hi_.right_lk9025_ecoder_zero) / 65535.0f) )
                        * 2 * PI * DRIVER_RADIUS;    
    need_data_form_hi_.right_lk9025_ecoder_last = need_data_form_hi_.right_lk9025_pos;
    need_data_form_hi_.left_lk9025_ecoder_last = need_data_form_hi_.left_lk9025_pos;
    // std::cout << "right totla dis:" << need_data_form_hi_.right_leg_total_dis << std::endl;
    // std::cout << "right velocity:" << need_data_form_hi_.right_lk9025_vel << std::endl;
    // std::cout << "right dis:" << need_data_form_hi_.right_lk9025_pos << std::endl;
    // std::cout << "right cnt -----!" << need_data_form_hi_.right_lk9025_circle_cnt << std::endl;
    // std::cout << "left totla dis:" << need_data_form_hi_.left_leg_total_dis << std::endl;
    // std::cout << "left velocity:" << need_data_form_hi_.left_lk9025_vel << std::endl;
    // std::cout << "left dis:" << need_data_form_hi_.left_lk9025_pos << std::endl;
    // std::cout << "left cnt -----!" << need_data_form_hi_.left_lk9025_circle_cnt << std::endl;
#else
    need_data_form_hi_.left_leg_total_dis = ((need_data_form_hi_.left_lk9025_pos)/(2 * PI)) * 2 * PI * DRIVER_RADIUS;
    need_data_form_hi_.right_leg_total_dis = ((need_data_form_hi_.right_lk9025_pos)/(2 * PI)) * 2 * PI * DRIVER_RADIUS;
    // std::cout << "left totla dis:      " << need_data_form_hi_.left_leg_total_dis << std::endl;
    // std::cout << "right totla dis:      " << need_data_form_hi_.right_leg_total_dis << std::endl;
#endif


}
/// @brief pass the k to controller
/// @param K the value of K
/// @param index the controller index
void WheelBalancingController::setLegLQRGain(MatrixXd K,uint8_t index)
{
    leg_balance_controller_[index].K = K;
    
    // std::cout << leg_balance_controller_[index].K << std::endl;
}

void WheelBalancingController::setLegLQRGain(MatrixXd K)
{
    balance_controller_.K = K;
}

void WheelBalancingController::updateXdes(uint8_t index)
{
    if(index == 0)
    {
        left_destination_.x = 0.0f;
        left_destination_.x_dot = 0.0f;
        left_destination_.theta_now = 0.0f;
        left_destination_.theta_dot = 0.0f;
        left_destination_.fai = 0.0f;
        left_destination_.fai_dot = 0.0f;
    }
    if(index == 1)
    {
        right_destination_.x = 0.0f;
        right_destination_.x_dot = 0.0f;
        right_destination_.theta_now = 0.0f;
        right_destination_.theta_dot = 0.0f;
        right_destination_.fai = 0.0f;
        right_destination_.fai_dot = 0.0f;
    }
}

void WheelBalancingController::updateXdes(void)
{
        state_var_tar_.x = 0.0f;
        state_var_tar_.x_dot = 0.0f;
        state_var_tar_.theta_now = 0.0f;
        state_var_tar_.theta_dot = 0.0f;
        state_var_tar_.fai = 0.0f;
        state_var_tar_.fai_dot = 0.0f;
}

void WheelBalancingController::setLegLQRXd(uint8_t index)
{
    if(index == 0)
    {
        leg_balance_controller_[index].X_d << left_destination_.x
        ,left_destination_.x_dot
        ,left_destination_.theta_now
        ,left_destination_.theta_dot
        ,left_destination_.fai
        ,left_destination_.fai_dot;
        //  std::cout <<"X_d:"<< leg_balance_controller_[index].X_d << std::endl;
    }
    if(index == 1)
    {
        leg_balance_controller_[index].X_d << right_destination_.x
        ,right_destination_.x_dot
        ,right_destination_.theta_now
        ,right_destination_.theta_dot
        ,right_destination_.fai
        ,right_destination_.fai_dot;
    }
}

void WheelBalancingController::setLegLQRXd(void)
{
        balance_controller_.X_d << state_var_tar_.x_dot
        ,state_var_tar_.theta_dot
        ,state_var_tar_.fai_dot
        ,state_var_tar_.x
        ,state_var_tar_.theta_now
        ,state_var_tar_.fai;
}

void WheelBalancingController::updateX(uint8_t index)
{
    pitch_now_ = need_data_form_hi_.pitch;
    if(index == 0)
    {
        left_set_feedback_.x = (need_data_form_hi_.left_leg_total_dis);//synthsis of velocities.        //m
        left_set_feedback_.x_dot = need_data_form_hi_.left_leg_dis_dot;//velocities of left lk9025              //m/s
        left_set_feedback_.theta_now = (PI/2) - left_five_bar_->exportBarLength()->q0 + pitch_now_;//           //rad
        left_set_feedback_.theta_dot = (left_set_feedback_.theta_now - left_set_feedback_.theta_last)/0.002f;  //rad/s
        left_set_feedback_.fai = pitch_now_;                                                                    //rad
        left_set_feedback_.fai_dot = need_data_form_hi_.wy;                                                     //rad/s
        // std::cout << "x" << left_set_feedback_.x << std::endl;
        // std::cout << "x_dot" << left_set_feedback_.x_dot << std::endl;
        // std::cout << "fai_now" << "   " << left_set_feedback_.fai * ( 180 / PI) << std::endl;
        // std::cout << "theta_now" << "   " << left_set_feedback_.theta_now * (180 / PI) << std::endl;
        // std::cout << "theta_dot" << "   " << left_set_feedback_.theta_dot << std::endl;
    }
    if(index == 1)
    {
        right_set_feedback_.x = (need_data_form_hi_.right_leg_total_dis);//synthsis of velocities.        //m
        right_set_feedback_.x_dot = need_data_form_hi_.right_leg_dis_dot;//velocities of left lk9025              //m/s
        right_set_feedback_.theta_now = (PI/2) - right_five_bar_->exportBarLength()->q0 + pitch_now_;//           //rad
        right_set_feedback_.theta_dot = (right_set_feedback_.theta_now - right_set_feedback_.theta_last)/0.002f;  //rad/s
        right_set_feedback_.fai = pitch_now_;                                                                    //rad
        right_set_feedback_.fai_dot = need_data_form_hi_.wy;   
    }
    left_set_feedback_.theta_last = left_set_feedback_.theta_now;
    right_set_feedback_.theta_last = right_set_feedback_.theta_now;
    pitch_last_ = pitch_now_;
}

void WheelBalancingController::updateX(void)
{
    pitch_now_ = need_data_form_hi_.pitch;

    state_var_now_.x = (need_data_form_hi_.right_leg_total_dis + need_data_form_hi_.left_leg_total_dis) / 2;//synthsis of velocities.        //m
    state_var_now_.x_dot = (need_data_form_hi_.right_leg_dis_dot + need_data_form_hi_.left_leg_dis_dot) / 2;//velocities of left lk9025              //m/s
    state_var_now_.theta_now = (PI/2) - right_five_bar_->exportBarLength()->q0 + pitch_now_;//           //rad
    state_var_now_.theta_dot = (state_var_now_.theta_now - state_var_now_.theta_last)/0.002f;  //rad/s
    state_var_now_.fai = pitch_now_;                                                                    //rad
    state_var_now_.fai_dot = need_data_form_hi_.wy;  

    state_var_now_.theta_last = state_var_now_.theta_now;
    pitch_last_ = pitch_now_; 
}

void WheelBalancingController::setLegLQRX(uint8_t index)
{
    if(index == 0)
    {
        leg_balance_controller_[index].X << left_set_feedback_.x
        ,left_set_feedback_.x_dot
        ,left_set_feedback_.theta_now
        ,left_set_feedback_.theta_dot
        ,left_set_feedback_.fai
        ,left_set_feedback_.fai_dot;
        // std::cout << leg_balance_controller_[index].X << std::endl;
    }
    if(index == 1)
    {
        leg_balance_controller_[index].X << right_set_feedback_.x
        ,right_set_feedback_.x_dot
        ,right_set_feedback_.theta_now
        ,right_set_feedback_.theta_dot
        ,right_set_feedback_.fai
        ,right_set_feedback_.fai_dot;
    }    
}

void WheelBalancingController::setLegLQRX(void)
{
        balance_controller_.X << state_var_now_.x_dot
        ,state_var_now_.theta_dot
        ,state_var_now_.fai_dot
        ,state_var_now_.x
        ,state_var_now_.theta_now
        ,state_var_now_.fai;
}

void WheelBalancingController::calclegLQRU(uint8_t index)
{ 
    if(index == 0)
    {
    leg_balance_controller_[index].U = -leg_balance_controller_[index].K * 
                    (leg_balance_controller_[index].X_d.transpose() - leg_balance_controller_[index].X.transpose());

        send_data_.left_tau_w = -leg_balance_controller_[index].U(0,0); // adjust motor rotation dirextion.
        // std::cout << "1 tau_w" << leg_balance_controller_[index].U(0,0) << std::endl;
        // std::cout << "2 tau_Tp" << leg_balance_controller_[index].U(1,0) << std::endl;
    }
    if(index == 1)
    {
        leg_balance_controller_[index].U = -leg_balance_controller_[index].K * 
            (leg_balance_controller_[index].X_d.transpose() - leg_balance_controller_[index].X.transpose());
        send_data_.right_tau_w = -leg_balance_controller_[index].U(0,0); // adjust motor rotation dirextion.
        // std::cout << "1 tau_w" << leg_balance_controller_[index].U(0,0) << std::endl;
    }        
}

void WheelBalancingController::calclegLQRU(void)
{
    balance_controller_.U = balance_controller_.K * 
                    (balance_controller_.X_d.transpose() - balance_controller_.X.transpose());
        send_data_.left_tau_w = balance_controller_.U(0,0); // adjust motor rotation dirextion.
        send_data_.right_tau_w = balance_controller_.U(0,0); // adjust motor rotation dirextion.
}

void WheelBalancingController::initLQRParam(void)
{
        left_lqr_ -> A = left_leg_lqr_param_.A_;
        left_lqr_ -> B = left_leg_lqr_param_.B_;
        left_lqr_ -> Q = left_leg_lqr_param_.Q_;
        left_lqr_ -> R = left_leg_lqr_param_.R_;
        left_lqr_ -> K = left_leg_lqr_param_.K_;
        left_lqr_ -> P = left_leg_lqr_param_.P_;

        right_lqr_ -> A = right_leg_lqr_param_.A_;
        right_lqr_ -> B = right_leg_lqr_param_.B_;
        right_lqr_ -> Q = right_leg_lqr_param_.Q_;
        right_lqr_ -> R = right_leg_lqr_param_.R_;
        right_lqr_ -> K = right_leg_lqr_param_.K_;
        right_lqr_ -> P = right_leg_lqr_param_.P_;

        lqr_ -> A = leg_lqr_param_.A_;
        lqr_ -> B = leg_lqr_param_.B_;
        lqr_ -> Q = leg_lqr_param_.Q_;
        lqr_ -> R = leg_lqr_param_.R_;
        lqr_ -> K = leg_lqr_param_.K_;
        lqr_ -> P = leg_lqr_param_.P_;
        // std::cout <<  left_lqr_ -> A << std::endl;
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
    // Lookup the wx state interface
    const auto wx_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == "wx";
    });
    if (wx_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s position state interface not found", joint_name.c_str());
        return nullptr;
    }
    // Lookup the wy state interface
    const auto wy_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == "wy";
    });
    if (wy_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s position state interface not found", joint_name.c_str());
        return nullptr;
    }
    // Lookup the wz state interface
    const auto wz_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == "wz";
    });
    if (wz_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s position state interface not found", joint_name.c_str());
        return nullptr;
    }

    // Lookup the ax state interface
    const auto ax_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == "ax";
    });
    if (ax_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s position state interface not found", joint_name.c_str());
        return nullptr;
    }


    // Lookup the ay state interface
    const auto ay_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == "ay";
    });
    if (ay_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s position state interface not found", joint_name.c_str());
        return nullptr;
    }


    // Lookup the az state interface
    const auto az_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == joint_name && interface.get_interface_name() == "az";
    });
    if (az_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s position state interface not found", joint_name.c_str());
        return nullptr;
    }

    // Create the ImuHandle instance
    return std::make_shared<ImuHandle>(std::ref(*pitch_state),
                                        std::ref(*yaw_state),
                                        std::ref(*roll_state),
                                        std::ref(*wx_state),
                                        std::ref(*wy_state),
                                        std::ref(*wz_state),
                                        std::ref(*ax_state),
                                        std::ref(*ay_state),
                                        std::ref(*az_state));
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
