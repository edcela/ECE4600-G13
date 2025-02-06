#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "Glove.h"
using namespace std::chrono_literals;  // This will allow us to use time literals like "1s", "500ms", etc.
                                       

//  The Node
class GestureNode : public rclcpp::Node
{
    public:
        GestureNode() : Node("gesture_node")
        {
            //  Subscribe to the Glove Topic 
            subscription_ = this->create_subscription<std_msgs::msg::UInt8>
            (
                "/esp32_glove",
                10,
                std::bind(&GestureNode::listener_callback, this, std::placeholders::_1)
            );

            //  Publish to the Drone Topic
            publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/esp32_drone", 10);

            // Timer: Publishing the telemetry message every second
            timer_ = this->create_wall_timer(
                500ms,  // 500 ms
                std::bind(&GestureNode::publish_msg, this));

            RCLCPP_INFO(this->get_logger(), "GestureNode node has been started.");
        }


    private:
        //  Listerner Callback Function
        void listener_callback(const std_msgs::msg::UInt8::SharedPtr msg)
        {
            //  Access the data from the received message 
            uint8_t data = msg->data;

            //  Determine Command/Agent Here
            //  CONVERSIONS
            //
            //
            //
            //
            //  CONVERSIONS

            //  Log the calculation 
            RCLCPP_INFO(this->get_logger(), "Received: [%u], Result: %d", static_cast<unsigned int>(data), result);

            RCLCPP_INFO(this->get_logger(), "Published: %d", result);
        }

        void publish_msg()
        {
            std_msgs::msg::UInt8MultiArray gesture_msg;
            gesture_msg.data = {latest_agentid_, latest_cmd_}; 

            RCLCPP_INFO(this->get_logger(), "Published Gesture message. Message: Agent ID: %d, Command: %d", gesture_msg.data[0], gesture_msg.data[1]);

            publisher_->publish(gesture_msg); 
        }
        
        //  Subscriber and Publisher
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        uint8_t latest_agentid_ = 0;
        uint8_t latest_cmd_ = 0;
        int result = 0;
        int status = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GestureNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
