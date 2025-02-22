//Base Docker Version of Glove Header File
//Only includes Gesture components of the Header File

#include "Glove.h"
#include <algorithm>

//Gesture class
//This class is now repurposed to be a means of categorizing gestures and comparing them to the current gesture
//This class also includes the command equivalent of the assigned gesture
gesture::gesture() : orientation(0), 
            flexion(0), 
            gestureID(0), 
            multipleOrientations(false),
            multipleFlexionStates(false), 
            assignedDroneCommand(CMD_INVALID),
            droneID(0)
            {}


gesture::~gesture() {}       //deconstructor

void gesture::updateID(uint8_t ID){
    gestureID = ID;
    flexion = ID && 0xF8;
    orientation = ID && 0x07;
}

//Get the sensor data of the gesture
uint8_t gesture::getGestureID() const {
    return gestureID;
}

//Set the BASE orientation to be used for IDing a gesture.
void gesture::setOrientation(uint8_t inputOrientation){
    gestureID = gestureID & 0xF8;           //clear last 3 bits

    gestureID = gestureID + inputOrientation;    //replaces the last three bits with the new orientation
    orientation = inputOrientation;
}

//If a gesture is valid for multiple SPECIFIC BUT NOT ALL orientations, add them using this function.
void gesture::addOrientation(uint8_t inputOrientation){
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
uint8_t gesture::getOrientation(){
    return orientation;
}

//Set the flexion states of the fingers for this gesture. The order is thumb, index, middle, ring, pinky.
void gesture::setFingerStates(uint8_t thumb, uint8_t index, uint8_t middle, uint8_t ring, uint8_t pinky) {
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
void gesture::addFingerStates(uint8_t thumb, uint8_t index, uint8_t middle, uint8_t ring, uint8_t pinky){
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
uint8_t gesture::getFingerStates() {
    return flexion;
}

void gesture::assignDroneCommand(DroneCommand command){
    assignedDroneCommand = command;
}

DroneCommand gesture::getDroneCommand(){
    return assignedDroneCommand;
}

void gesture::setDroneID(int droneNum){
    droneID = droneNum;
}

int gesture::getDroneID(){
    return droneID;
}

//Check if the gesture matches the current gesture
bool gesture::checkGesture(uint8_t currentGesture) const{
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
bool gesture::operator < (const gesture& otherGesture) const {
    return gestureID < otherGesture.getGestureID();
}

bool gesture::operator > (const gesture& otherGesture) const {
    return gestureID > otherGesture.getGestureID();
}

bool gesture::operator == (const gesture& otherGesture) const {
    return this->checkGesture(otherGesture.getGestureID());
}

void gesture::operator = (gesture& replacementGesture){
    this->gestureID = replacementGesture.getGestureID();
    this->flexion = replacementGesture.getFingerStates();
    this->orientation = replacementGesture.getOrientation();
}

//removes all gesture information
void gesture::clearGesture(){
    gestureID = 0;
    flexion = 0;
    orientation = 0;

    assignedDroneCommand = CMD_INVALID;
    multipleFlexionStates = false;
    multipleOrientations = false;

    allowedFlexionStates.clear();
    allowedOrientations.clear();

}

//Standardizes gesture command to Neutral
void gesture::clearCommand(){
    assignedDroneCommand = CMD_INVALID;
}

//Creates Gesture Bank
GestureBank::GestureBank() {
    std::vector<gesture> bank;
}

GestureBank::~GestureBank() {}

//Adds gesture to the gesture bank
void GestureBank::addGesture(gesture newGesture){
    bank.push_back(newGesture);
}

//Returns an iterator for the gesture if its found in the bank
std::vector<gesture>::iterator GestureBank::findGesture(uint8_t gestureToFindID){
    std::vector<gesture>::iterator it = std::find_if(bank.begin(), bank.end(), [&gestureToFindID](const gesture& g){
        return g.checkGesture(gestureToFindID);   //equality condition
    });

    return it;
}