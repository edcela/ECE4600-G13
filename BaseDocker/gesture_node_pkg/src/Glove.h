#ifndef GLOVE_H
#define GLOVE_H

#include <vector>
#include <string>
#include <iostream>
#include <stdint.h>

//FOR UNIVERSAL IDS OF GESTURE MESSAGE PACKET
// Defines for the different hand orientations
#define FINGER_UP   0b00000001
#define FINGER_DOWN 0b00000110
#define THUMB_UP    0b00000010
#define THUMB_DOWN  0b00000101
#define PALM_UP     0b00000100
#define PALM_DOWN   0b00000011

// Defines for the different finger flexion states
#define FLEX 0x00
#define EXTD 0x01

// Defines for drone identifiers
#define DRN_1 0b00000001
#define DRN_2 0b00000010
#define DRN_3 0b00000100
#define DRN_4 0b00001000
#define DRN_5 0b00010000

//Commands for controlling the drone
typedef enum {
    CMD_LEFT                = 1,
    CMD_RIGHT               = 2,
    CMD_FORWARD             = 3,
    CMD_BACK                = 4,
    CMD_ASCEND              = 5,
    CMD_DESCEND             = 6,
    CMD_REGULAR_SHUTOFF     = 7,
    CMD_EMERGENCY_SHUTOFF   = 8,
    CMD_INVALID             = 0   
} DroneCommand;

//Gesture class
//This class is now repurposed to be a means of categorizing gestures and comparing them to the current gesture
//This class also includes the command equivalent of the assigned gesture
class gesture {
public:
    //Create a gesture object with a name and a set of finger states.
    gesture() : orientation(0), 
                flexion(0), 
                gestureID(0), 
                multipleOrientations(false),
                multipleFlexionStates(false), 
                assignedDroneCommand(CMD_INVALID){}
    
    ~gesture() {}       //deconstructor

    void updateID(uint8_t ID){
        gestureID = ID;
        flexion = ID && 0xF8;
        orientation = ID && 0x07;
    }

    //Set the BASE orientation to be used for IDing a gesture.
    void setOrientation(uint8_t inputOrientation){
        gestureID = gestureID & 0xF8;           //clear last 3 bits

        gestureID = gestureID + inputOrientation;    //replaces the last three bits with the new orientation
        orientation = inputOrientation;
    }

    //If a gesture is valid for multiple SPECIFIC BUT NOT ALL orientations, add them using this function.
    void addOrientation(uint8_t inputOrientation){
        if(!multipleOrientations){
            multipleOrientations = true;

            allowedOrientations.push_back(this->orientation);
            allowedOrientations.push_back(inputOrientation);
        }
        else{
            allowedOrientations.push_back(inputOrientation);
        }
    }

    //Returns the orientation of the gesture. If multiple orientations are allowed, only the first orientation is returned.
    uint8_t getOrientation(){
        return orientation;
    }

    //Set the flexion states of the fingers for this gesture. The order is thumb, index, middle, ring, pinky.
    void setFingerStates(uint8_t thumb, uint8_t index, uint8_t middle, uint8_t ring, uint8_t pinky) {
        uint8_t fingerHandle = 0x00;    //0b00000000   
        gestureID = gestureID & 0x07;   //clear first 5 bits

        fingerHandle = fingerHandle + (index << 7);
        fingerHandle = fingerHandle + (middle << 6);
        fingerHandle = fingerHandle + (ring << 5);
        fingerHandle = fingerHandle + (pinky << 4);
        fingerHandle = fingerHandle + (thumb << 3);

        gestureID = gestureID + fingerHandle;
        flexion = fingerHandle;
    }

    //If a gesture requires multiple possible finger states, add them using this function.
    void addFingerStates(uint8_t thumb, uint8_t index, uint8_t middle, uint8_t ring, uint8_t pinky){
        uint8_t fingerHandle = 0x00;    //0b11111000

        fingerHandle = fingerHandle + (index << 7);
        fingerHandle = fingerHandle + (middle << 6);
        fingerHandle = fingerHandle + (ring << 5);
        fingerHandle = fingerHandle + (pinky << 4);
        fingerHandle = fingerHandle + (thumb << 3);

        if(!multipleFlexionStates){
            multipleFlexionStates = true;

            allowedFlexionStates.push_back(flexion);
            allowedFlexionStates.push_back(fingerHandle);
        }
        else{
            allowedFlexionStates.push_back(fingerHandle);
        }
    }

    //Returns flexion states. As in the gestureID of this gesture with orientation omitted.
    uint8_t getFingerStates() {
        return flexion;
    }

    //Get the sensor data of the gesture
    uint8_t getGestureID() const {
        return gestureID;
    }

    void assignDroneCommand(DroneCommand command){
        assignedDroneCommand = command;
    }

    DroneCommand getDroneCommand(){
        return assignedDroneCommand;
    }

    //Check if the gesture matches the current gesture
    bool checkGesture(uint8_t currentGesture) const{
        //It is implemented that certain gestures can have specific allowed states.
        //So the cases where multiple states are allowed are handled differently than if
        //a gesture only has one allowed state.

        //If the gesture does not have an assigned orientation, only the finger states are checked.
        if(orientation == 0){
          if(!multipleFlexionStates){
            if((currentGesture & 0xF8) == flexion){
                return true;
            }
            else{
                return false;
            }
          }
          else if(multipleFlexionStates){
            for (int i = 0; i < allowedFlexionStates.size(); i++) {
              if ((currentGesture & 0xF8) == allowedFlexionStates[i]) {
                return true;
              }
            }
            return false;
          }
        }

        else{
        //If the gesture has assigned orientation, the function will check depending the following
        //four cases:   1. If the gesture has only one orientation and only one flexion state
        //              2. If the gesture has multiple orientations and only one flexion state
        //              3. If the gesture has only one orientation and multiple flexion states
        //              4. If the gesture has multiple orientations and multiple flexion states
          if(!(multipleFlexionStates)){
              if(!(multipleOrientations)){
                  if (currentGesture == gestureID) {
                      return true;
                  }
                  else {
                      return false;
                  }
              }
              else if(multipleOrientations){
                  for (int i = 0; i < allowedOrientations.size(); i++) {
                      if (currentGesture == (flexion + allowedOrientations[i])) {
                          return true;
                      }
                  }
                  return false;
              }
          }

          else if(multipleFlexionStates){
              if(!(multipleOrientations)){
                  for (int i = 0; i < allowedFlexionStates.size(); i++) {
                      if (currentGesture == (allowedFlexionStates[i] + orientation)) {
                          return true;
                      }
                  }
                  return false;
              }
              else if(multipleOrientations){
                  for (int i = 0; i < allowedFlexionStates.size(); i++) {
                      for (int j = 0; j < allowedOrientations.size(); j++) {
                          if (currentGesture == (allowedFlexionStates[i] + allowedOrientations[j])) {
                              return true;
                          }
                      }
                  }
                  return false;
              }
          }
        }
    }

    //this is used for sorting gestures in a set
    bool operator < (const gesture& otherGesture) const {
        return gestureID < otherGesture.getGestureID();
    }

    bool operator > (const gesture& otherGesture) const {
        return gestureID > otherGesture.getGestureID();
    }

    bool operator == (const gesture& otherGesture) const {
        return this->checkGesture(otherGesture.getGestureID());
    }

    //removes all gesture information
    void clearGesture(){
        gestureID = 0;
        flexion = 0;
        orientation = 0;
    }

    void clearCommand(){
        assignedDroneCommand = CMD_INVALID;
    }

private:
    uint8_t gestureID;                  //Sensor data formatted such that the first 5 bits are the finger states and the last 3 bits are the hand orientation
    DroneCommand assignedDroneCommand;  //The DRONE command that is assigned to the gesture

    uint8_t flexion;
    bool multipleFlexionStates;
    std::vector<uint8_t> allowedFlexionStates;
    
    uint8_t orientation;        //This is the base orientation of the gesture (the first orientation added)
    bool multipleOrientations;
    std::vector<uint8_t> allowedOrientations;
};

#endif // GLOVE_H
