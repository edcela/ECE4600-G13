#include <Arduino.h>
#include <stdio.h>
#include <HardwareSerial.h>
#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h> 
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/subscription.h>
#include <std_msgs/msg/u_int8_multi_array.h>

//  Definitions
#define TX_PIN 17 //  Pin 2
#define RX_PIN 16 //  Pin 4
#define UART_BAUD_RATE 115200
#define ALL_AGENTS 0
#define AGENT_ID 0b00000001    //Change this to whatever bit this bot is assigned for

HardwareSerial agentSerial(1);

/* Hotspot */
char* ssid = "INSERT HOTSPOT SSID HERE";
char* pass = "INSERT HOTSPOT PASSWORD HERE";
char* agent_ip = "INSERT MAIN COMPUTER IP";

char* agent_port = "8888";

// ROS 2 topic message for subscribing
rcl_subscription_t subscriber;

std_msgs__msg__UInt8MultiArray incoming_msg;

rcl_allocator_t allocator;
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;

// Callback function for received messages
void subscription_callback(const void *msg_in) 
{
    const std_msgs__msg__UInt8MultiArray *msg = (const std_msgs__msg__UInt8MultiArray *)msg_in;
    Serial.println("Message Received: ");
    for (size_t i = 0; i < msg->data.size; i++)
    {
        Serial.printf("Data[%d] = %d\n", i, msg->data.data[i]);
    }

    delay(200);

    //    CHANGE THIS TO WORK WITH THE BITWISE
    int IDCheck = 0;
    IDCheck = AGENT_ID & data[0];
    
    if (msg->data.data[0] == 0 || IDCheck != 0)
    {
      Serial.println("Agent will send: ");
      //Serial.print(msg->data.data[1]);
      agentSerial.print((int)msg->data.data[1]);
    }
}

void setup()
{
    //  Initialize Serial for debugging 
    Serial.begin(UART_BAUD_RATE);
    agentSerial.begin(UART_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
    delay(1000);

    //  Initialize micro-ROS transport
    set_microros_wifi_transports(ssid, pass, agent_ip, atoi(agent_port));

    //  Initialize the Int32MultiArray
    if (std_msgs__msg__UInt8MultiArray__init(&incoming_msg)) 
    {
        Serial.println("Int32MultiArray initialized");
    } 
    else 
    {
        Serial.println("Failed Int32MultiArray initialization");
    }

    // Allocate space for array data manually (if known size)
    incoming_msg.data.data = (uint8_t *)malloc(2 * sizeof(uint8_t));
    incoming_msg.data.size = 0;
    incoming_msg.data.capacity = 2;

    //  Ping the microROS Agent 
    Serial.print("Pinging micro-ROS Agent... ");
    if (rmw_uros_ping_agent(2000, 10)) 
    {  // Wait up to 2 seconds, try 10 times
        Serial.println("Connected to micro-ROS Agent");
    } else 
    {
        Serial.println("Failed to connect to micro-ROS Agent");
    }

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
    size_t domain_id = 42;
    ret = rcl_init_options_set_domain_id(&init_options, domain_id);
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
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
        "/esp32_drone"
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
    delay(10);
}
