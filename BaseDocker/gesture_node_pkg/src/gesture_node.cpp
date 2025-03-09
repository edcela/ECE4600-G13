#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "Glove.h"
using namespace std::chrono_literals;  // This will allow us to use time literals like "1s", "500ms", etc.

    //  Establish Gesture Bank for Robot Commands
std::vector<gesture> initializeCommandBank()
    {
        std::vector<gesture> bank;

        gesture pointUp;            //drone go up
        gesture pointDown;          //drone go down
        gesture pointForward;       //drone go forward
        gesture thumbsBack;         //drone go backward
        gesture thumbsLeft;         //drone go left
        gesture thumbsRight;        //drone go right

        pointUp.setOrientation(FINGER_UP);
        pointUp.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);
        pointUp.assignDroneCommand(CMD_ASCEND);
        bank.push_back(pointUp);

        pointDown.setOrientation(FINGER_DOWN);
        pointDown.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);
        pointDown.assignDroneCommand(CMD_DESCEND);
        bank.push_back(pointDown);

        pointForward.setOrientation(THUMB_UP);
        pointForward.addOrientation(PALM_UP);
        pointForward.addOrientation(PALM_DOWN);
        pointForward.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);
        pointForward.assignDroneCommand(CMD_FORWARD);
        bank.push_back(pointForward);

        thumbsBack.setOrientation(FINGER_UP);
        thumbsBack.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);
        thumbsBack.assignDroneCommand(CMD_BACK);
        bank.push_back(thumbsBack);

        thumbsLeft.setOrientation(PALM_DOWN);
        thumbsLeft.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);
        thumbsLeft.assignDroneCommand(CMD_LEFT);
        bank.push_back(thumbsLeft);

        thumbsRight.setOrientation(PALM_UP);
        thumbsRight.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);
        thumbsRight.assignDroneCommand(CMD_RIGHT);
        bank.push_back(thumbsRight);

        std::sort(bank.begin(), bank.end());

        return bank;
    }

    //  Establish Gesture Bank for Robot Selection
std::vector<gesture> initializeSelectBank()
    {
        std::vector<gesture> bank;

        gesture oneGesture;
        gesture twoGesture;
        gesture threeGesture;
        gesture fourGesture;
        gesture thumbsDown;         //reject
        gesture thumbsUp;           //select
        gesture erase;                //clear selectedAgents

        oneGesture.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);
        oneGesture.addFingerStates(FLEX,FLEX,FLEX,FLEX,EXTD);
        oneGesture.setOrientation(FINGER_UP);
        oneGesture.setDroneID((uint8_t)1);
        bank.push_back(oneGesture);

        twoGesture.setFingerStates(FLEX,EXTD,EXTD,FLEX,FLEX);
        twoGesture.addFingerStates(FLEX,FLEX,FLEX,EXTD,EXTD);
        twoGesture.setOrientation(FINGER_UP);
        twoGesture.setDroneID((uint8_t)2);
        bank.push_back(twoGesture);

        threeGesture.setFingerStates(FLEX, EXTD, EXTD, EXTD, FLEX);
        threeGesture.addFingerStates(FLEX, FLEX, EXTD, EXTD, EXTD);
        threeGesture.setOrientation(FINGER_UP);
        threeGesture.setDroneID((uint8_t)3);
        bank.push_back(threeGesture);

        fourGesture.setFingerStates(FLEX, EXTD, EXTD, EXTD, EXTD);
        fourGesture.setOrientation(FINGER_UP);
        fourGesture.setDroneID((uint8_t)4);
        bank.push_back(fourGesture);

        thumbsDown.setOrientation(THUMB_DOWN);
        thumbsDown.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);
        bank.push_back(thumbsDown);

        thumbsUp.setOrientation(THUMB_UP);
        thumbsUp.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);
        bank.push_back(thumbsUp);

        erase.setOrientation(FINGER_DOWN);
        erase.setFingerStates(EXTD,FLEX,EXTD,EXTD,FLEX);
        erase.addFingerStates(FLEX,FLEX,EXTD,EXTD,FLEX);
        bank.push_back(erase);

        std::sort(bank.begin(), bank.end());

        return bank;
    }

//  The Node
class GestureNode : public rclcpp::Node
{

    public:
        GestureNode() : Node("gesture_node")
        {

            //Initialize configuration for mode toggle gesture, The gesture is "Rock and Roll"
            toggle.setOrientation(FINGER_UP);
            toggle.setFingerStates(FLEX,EXTD,FLEX,FLEX,EXTD);
            toggle.addFingerStates(EXTD,EXTD,FLEX,FLEX,EXTD);

            toggleLock = false;
            agentLock = false;
          
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

            //Toggle between agent select and control mode
            //This toggle MUST BE DISABLED after a toggle until another command enables it
            if(toggle.checkGesture(data) && !toggleLock)
            {
                mode_ = (mode_ == PHASE_SELECTION) ? PHASE_CONTROL : PHASE_SELECTION;
                toggleLock = true;  //locks the toggle until another gesture is done, this prevents repetitive calling when user does toggle gesture
                if(mode_ == PHASE_SELECTION){
                    RCLCPP_INFO(this->get_logger(), "Switching to Agent Select Mode");
                }
                else if(mode_ == PHASE_Control){
                    RCLCPP_INFO(this->get_logger(), "Switching to Agent Control Mode");
                }
            }

            //Select Phase Logic
            else if(mode_ == PHASE_SELECTION)
            {
                std::vector<gesture>::iterator it = std::find_if(selectGestureBank.begin(), selectGestureBank.end(), [data](const gesture& g){
                    return g.checkGesture(data);   //equality condition, this checks if there is a matching ID with the gestures in the gesture bank
                });
                if(it != selectGestureBank.end()){
                    //Case 1: Thumbs up -> update latest_agentid_ with selectedAgents
                    if(it->getGestureID() == 0b00001010){
                        RCLCPP_INFO(this->get_logger(), "Agent %d added to controlled agents!", selectedAgent);
                        latest_agentid_ |= selectedAgent;    //update latest_agentid_
                        selectedAgent = 0;                    //reset selectedAgents back to 0
                        agentLock = false;
                    }
                    
                    //Case 2: Thumbs down -> update latest_agentid_ so selectedAgents are toggled off
                    if(it->getGestureID() == 0b00001101){
                        RCLCPP_INFO(this->get_logger(), "Agent %d removed from controlled agents!", selectedAgent);
                        latest_agentid_ &= ~selectedAgent;    //Toggle off selected agents, needs to be checked
                        selectedAgent = 0;
                        agentLock = false;
                    }

                    //Case 3: This weird gesture basically opposite of rock and roll gesture -> resets selectedAgent
                    if(it->getGestureID() == 0b01101110){
                        RCLCPP_INFO(this->get_logger(), "Selected agents cleared");
                        selectedAgent = 0;
                        agentLock = false;
                    }
                        
                    //Case 4: Numbers -> update selectedAgents for that corresponding number
                    else{
                        if(!agentLock){
                            selectedAgent = it->getDroneID();    //potential error, potential solution: dereference the iterator (*it)
                            agentLock = true;
                            RCLCPP_INFO(this->get_logger(), "Agent %u selected",selectedAgent);
                        }
                        else{
                            RCLCPP_INFO(this->get_logger(), "Confirm what you want to do with Agent %u first!",selectedAgent);
                        }
                    }

                toggleLock = false;    //resets toggleLock
                }
            }
            
            //Control Phase Logic
            else if(mode_ == PHASE_CONTROL)
            {
                std::vector<gesture>::iterator it = std::find_if(droneGestureBank.begin(), droneGestureBank.end(), [data](const gesture& g){
                    return g.checkGesture(data); });  //equality condition, this checks if there is a matching ID with the gestures in the gesture bank
                
                if(it != droneGestureBank.end())
                {
                    latest_cmd_ = it->getDroneCommand();    //potential error, potential solution: dereference the iterator (*it)
                    toggleLock = false;
                }
                else
                {   //Default Case, for rc, CMD_INVALID should equal stop
                    latest_cmd_ = CMD_INVALID;
                }
            }

            //  Log the calculation 
            RCLCPP_INFO(this->get_logger(), "Received: [%u], Result: %d", static_cast<unsigned int>(data), result);
            RCLCPP_INFO(this->get_logger(), "Published: %d", result);
        }

        void publish_msg()
        {
            std_msgs::msg::UInt8MultiArray gesture_msg;
            gesture_msg.data = {latest_agentid_, latest_cmd_}; 
            if (mode_ == PHASE_CONTROL)
            {
                RCLCPP_INFO(this->get_logger(), "Published Gesture message. Message: Agent ID: %d, Command: %d", gesture_msg.data[0], gesture_msg.data[1]);
                publisher_->publish(gesture_msg); 
            }

        }
        
        //  Subscriber and Publisher
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;
        
        //  Update these when a new command or new agent_id is chosen 
        uint8_t latest_agentid_ = 0;
        uint8_t latest_cmd_ = 0;
        int result = 0;
        int status = 0;

        //  Glove Components 
        uint8_t selectedAgent;             //Buffer for agent select phase

        gesture toggle;                     //Toggle gesture for Select and Control Agent phases
        bool toggleLock;
        bool agentLock;                    //So that you have to confirm a selected agent before being able to select another

        phase mode_ = PHASE_SELECTION;      //Indicates what mode you are currently on. "phase" included in Glove.h
        //bool can_Publish = false;
                                          
        //Creates bank of gestures
        std::vector<gesture> droneGestureBank = initializeCommandBank();
        std::vector<gesture> selectGestureBank = initializeSelectBank();

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GestureNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
