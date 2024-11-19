#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <std_msgs/msg/string.h>

/* Wi-Fi Credentials SCHOOL
char* ssid = "uofm-guest";
char* pass = "";
char* agent_ip = "140.193.173.186";

*/

/* Wi-Fi Credentials HOME */
char* ssid = "SHAW-8660";
char* pass = "corral4479empty";
char* agent_ip = "10.0.0.42";

uint16_t agent_port = 8888;

// ROS 2 Variables 
rcl_subscription_t subscriber;
std_msgs__msg__String msg;
rcl_allocator_t allocator;
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;


// Callback function for received messages
void subscription_callback(const void *msg_in) {
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msg_in;
    Serial.print("Received message: ");
    Serial.println(msg->data.data);
}

// Setup
void setup()
{
    //  Initialize Serial for debugging 
    Serial.begin(115200);
    delay(1000);

    //  Attempt to connect to Wi-Fi
    WiFi.begin(ssid, pass);
    while(WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected");
    
    //  Initialize micro-ROS transport
    set_microros_wifi_transports(ssid, pass, agent_ip, agent_port);

    //  Initialize micro-ROS
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    
    //  Node Creation
    rclc_node_init_default(&node, "esp32_node", "", &support);

    //  Create a subscriber 
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/esp32_topic"
    );

    // Set the subscription callback
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    
    Serial.println("microROS setup complete!");
}

// Void 
void loop()
{
    Serial.println("Spinning...");

    //  Spin Executor
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    //  Delay to control publishing rate
    delay(1000);

}
