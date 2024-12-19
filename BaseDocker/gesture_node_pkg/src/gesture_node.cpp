#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "gesture_node_pkg/msg/gesture.hpp"

using namespace std::chrono_literals;  // This will allow us to use time literals like "1s", "500ms", etc.
                                       

//  The Node
class GestureNode : public rclcpp::Node
{
    public:
        GestureNode() : Node("gesture_node")
        {
            //  Suscribe to the Glove Topic 
            subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>
            (
                "esp32_glove",
                10,
                std::bind(&GestureNode::listener_callback, this, std::placeholders::_1)
            );

            //  Publish to the Drone Topic
            publisher_ = this->create_publisher<gesture_node_pkg::msg::Gesture>("esp32_drone", 10);

            // Timer: Publishing the telemetry message every second
            timer_ = this->create_wall_timer(
                1s,  // 1 second
                std::bind(&GestureNode::publish_msg, this));

            RCLCPP_INFO(this->get_logger(), "GestureNode node has been started.");
        }


    private:
        //  Listerner Callback Function
        void listener_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
        {
            //  Access the data from the received message 
            std::vector<int> data = msg->data;

            //  Gesture Control Control Here
            //
            //
            //  result = 
            //

            int result = 1;

            latest_cmd_ = 1;

            //  Log the calculation 
            RCLCPP_INFO(this->get_logger(), "Received: [%s], Result: %d", vector_to_string(data).c_str(), result);

            RCLCPP_INFO(this->get_logger(), "Published: %d", result);
        }

        void publish_msg()
        {
            auto gesture_msg = gesture_node_pkg::msg::Gesture();
            gesture_msg.agent_id = latest_agentid_;
            gesture_msg.command = latest_cmd_;

            publisher_->publish(gesture_msg); 

            RCLCPP_INFO(this->get_logger(), "Published Gesture message.");
        }

        //  Helper Function - convert vector to string 
        std::string vector_to_string(const std::vector<int32_t> &vec)
        {
            std::string result = "";

            for (size_t i = 0; i < vec.size(); i++)
            {
                result += std::to_string(vec[i]);
                if (i != vec.size()-1)
                    result += ", ";
            }

            return result;

        }
        
        //  Subscriber and Publisher
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
        rclcpp::Publisher<gesture_node_pkg::msg::Gesture>::SharedPtr publisher_;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        int latest_agentid_ = -1;
        int latest_cmd_ = -1;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GestureNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
