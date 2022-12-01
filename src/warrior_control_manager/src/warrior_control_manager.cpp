#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // Setup the controller manager node
    std::string manager_node_name = "controller_manager";
    std::shared_ptr<rclcpp::Executor> executor =std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto controller_manager_node = std::make_shared<controller_manager::ControllerManager>(executor, manager_node_name);

    std::thread cm_thread([controller_manager_node]() {
      // load controller_manager update time parameter
      int update_rate = 100;
      if (!controller_manager_node->get_parameter("update_rate", update_rate)) {
        throw std::runtime_error("update_rate parameter not existing or empty");
      }
      RCLCPP_INFO(controller_manager_node->get_logger(), "update rate is %d Hz", update_rate);

      while (rclcpp::ok()) {
        std::chrono::system_clock::time_point begin = std::chrono::system_clock::now();
        controller_manager_node->read();
        controller_manager_node->update();
        controller_manager_node->write();
        std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
        std::this_thread::sleep_for(
          std::max(
            std::chrono::nanoseconds(0),
            std::chrono::nanoseconds(1000000000 / update_rate) -
            std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin)));
      }
     });

    // Load the controllers
    std::vector<std::string> start_controllers;
    std::vector<std::string> stop_controllers;
    controller_manager_node->load_controller("test_controller", "warrior_controller/TestController");
    controller_manager_node->load_controller("wheel_balance_controller", "warrior_controller/WheelBalancingController");
    controller_manager_node->configure_controller("test_controller");
    controller_manager_node->configure_controller("wheel_balance_controller");
    start_controllers.push_back("test_controller");
    start_controllers.push_back("wheel_balance_controller");
    controller_manager_node->switch_controller(start_controllers, stop_controllers, 1, controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT);

    // Run the node(s)
    executor->add_node(controller_manager_node);
    executor->spin();
    cm_thread.join();
    // Exit
    rclcpp::shutdown();
    return 0;
}