#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Speed_est : public rclcpp::Node
{
public:
    Speed_est() : Node("Speed_est")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!";
        publisher_->publish(message);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Speed_est>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}