#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <memory>
#include "titanfall_ai/main_controller.hpp"

// Global pointer to main controller for signal handling
std::shared_ptr<titanfall_ai::MainController> g_main_controller = nullptr;

void signalHandler(int signal) {
    if (g_main_controller) {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Received signal %d, shutting down...", signal);
        g_main_controller->shutdown();
    }
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Set up signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        // Create main controller
        g_main_controller = std::make_shared<titanfall_ai::MainController>();
        
        // Initialize the system
        g_main_controller->initialize();
        
        // Start the system
        g_main_controller->start();
        
        // Run the main loop
        g_main_controller->run();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
        return 1;
    }
    
    // Cleanup
    if (g_main_controller) {
        g_main_controller->shutdown();
    }
    
    rclcpp::shutdown();
    return 0;
}
