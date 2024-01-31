#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_interfaces/msg/cone_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class LongitudinalControl : public rclcpp::Node {                           //class that holds fucntions to publish and hold data
    private:
        //declaration of 4 variables
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_velcoity;
        rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr path_subscription;
        double velocity;

        void publish_torque(custom_interfaces::msg::ConeArray path);              // function that only publishes torque (or other output such as acceleration)

        void velocity_estimation_callback(std_msgs::msg::String velocity);        //function to transfer data to the variable "velocity"

    public:
        LongitudinalControl();
};