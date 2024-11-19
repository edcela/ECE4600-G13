#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h> 
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>

/* Wi-Fi Credentials SCHOOL */
char* ssid = "uofm-guest";
char* pass = "";
char* agent_ip = "140.193.168.231";


uint16_t agent_port = 8888;

// ROS 2 topic message for subscribing
rcl_subscription_t subscriber;

std_msgs__msg__Int32 incoming_msg;

rcl_allocator_t allocator;
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;


// Callback function for received messages
void subscription_callback(const void *msg_in) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msg_in;
    Serial.print("Received message: ");
    Serial.println(msg->data);
}

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

    //  Ping the microROS Agent 
    Serial.print("Pinging micro-ROS Agent... ");
    if (rmw_uros_ping_agent(2000, 10)) 
    {  // Wait up to 2 seconds, try 10 times
        Serial.println("Connected to micro-ROS Agent");
    } else 
    {
        Serial.println("Failed to connect to micro-ROS Agent");
    }

    //  Print ESP32 Address
    Serial.println("ESP32 IP Address");
    Serial.println(WiFi.localIP());

    //  Initialize micro-ROS
    allocator = rcl_get_default_allocator();

    //  Initialize RCL init options 
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
    if (ret != RCL_RET_OK)
    {
        Serial.println("Failed to initialize RCL init options");
        return;
    }
    else
    {
        Serial.println("RCL init options initialized.");  
    }

    //  Set ROS DOMAIN ID Option to 
    ret = rcl_init_options_set_domain_id(&init_options, 42);
    if (ret != RCL_RET_OK)
    {
        Serial.println("\nFailed to set domain ID");
        return;
    }
    else
    {
        Serial.println("ROS_DOMAIN_ID Set.");  
    }
  
    // Initialize micro-ROS support with default options
    ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    if (ret != RCL_RET_OK) 
    {
        Serial.printf("\nFailed to initialize micro-ROS support, error %d\n", ret);
        return;
    }
    else
    {
        Serial.println("micro-ROS support initialized.");  
    }

    //  Node Creation
    ret = rclc_node_init_default(&node, "esp32_node", "", &support);
    if (ret != RCL_RET_OK)
    {
        Serial.printf("\nFailed to initialize node, error: %d\n", ret);  
        return;
    }
    else 
    {
        Serial.println("Node Created.");
    }

    //  Create a subscriber 
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/esp32_topic"
    );

    // Set the subscription callback
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &incoming_msg, &subscription_callback, ON_NEW_DATA);
       
    Serial.println("microROS setup complete!");
}

void loop()
{

    Serial.println("Spinning...");
    //  Spin Executor
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    //  Delay to control publishing rate
    delay(1000);

}
