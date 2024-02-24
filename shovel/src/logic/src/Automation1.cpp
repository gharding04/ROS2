#include <cmath>
#include <ctime>

#include "logic/Automation.hpp"
#include "logic/Automation1.hpp"

int left = 0;
int destX = 10, destY = 10;

/** @file
 *
 * @brief Defines functions used in Automation1.hpp
 * 
 * This function sets the wheel speed and spins to the right until the camera
 * sees the Aruco marker, then drives forward until the robot is less than a
 * meter away from the marker.
 * */

void Automation1::automate(){
    // Initially start with locating the Aruco marker
    // Turn slowly until it's seen

    if(robotState==ROBOT_IDLE){
        //setCameraSpeed(1.0);
        setDestPosition(destX, destY);
        robotState = LOCATE;
    }

    // TODO: Change this to align
    if(robotState==LOCATE){
        changeSpeed(0.15,-0.15);
        if(position.arucoVisible==true){
            if (abs(position.aruco_roll) < 90.0) {
                left = 1;
            } else { // dot on top
                left = -1;
            }
            RCLCPP_INFO(this->node->get_logger(), "Left: %d", left);
            setDestAngle(position.yaw + 90.0);
            changeSpeed(0,0);
            robotState=ALIGN;
        }
    }

    // After finding the Aruco marker, turn the bot to 
    // align with the arena
    if(robotState==ALIGN){
        RCLCPP_INFO(this->node->get_logger(), "Left: %d", left);
        if (!(position.yaw < this->destAngle+2 && position.yaw > this->destAngle-2)) {
            changeSpeed(0.15*left, -0.15*left);
        } 
        else {
            changeSpeed(0, 0);
            setStartPosition(this->search.row - std::ceil(position.z * 10), std::ceil(position.x * 10));
            aStar();
            RCLCPP_INFO(this->node->get_logger(), "Current Position: %d, %d", this->search.startX, this->search.startY);
            setGo();
            std::pair<int, int> initial = this->currentPath.top();
            this->currentPath.pop();
            setDestZ(initial.first);
            setDestX(initial.second);
            setDestAngle(getAngle());
            robotState = GO_TO_DIG_SITE;
        }
    }

    // After aligning with the arena, navigate to the 
    // excavation area
    if(robotState==GO_TO_DIG_SITE){
        RCLCPP_INFO(this->node->get_logger(), "GO_TO_DIG_SITE");
        RCLCPP_INFO(this->node->get_logger(), "ZedPosition.z: %f", this->position.z);
        RCLCPP_INFO(this->node->get_logger(), "Left: %d", left);
        //TODO: Take rotation of robot into account
        if(position.distance < normalDistance - 0.5 || position.distance > normalDistance + 0.5){
            changeSpeed(0, 0);
            robotState = OBSTACLE;
            previousState = GO_TO_DIG_SITE;
        }
        if (!(position.yaw < this->destAngle+2 && position.yaw > this->destAngle-2)) {
            if(position.yaw - this->destAngle > 180 || position.yaw - this->destAngle < 0){
                changeSpeed(0.15, -0.15);
            }
            else{
                changeSpeed(-0.15, 0.15);
            }
        } 
        else{ 
            if(abs(this->position.x) > abs(this->destX)){
                changeSpeed(0.0, 0.0);
                if(this->currentPath.empty()){
                    robotState = EXCAVATE;
                }
                else{
                    std::pair<int, int> current = this->currentPath.top();
                    this->currentPath.pop();
                    setDestZ(current.first);
                    setDestX(current.second);
                    setDestAngle(getAngle());
                }
            }
            else if(abs(this->position.x) > abs(this->destX) - 0.1){
                changeSpeed(0.1, 0.1);
            }
            else if(abs(this->position.x) > abs(this->destX) - 0.25){
                changeSpeed(0.15, 0.15);
            }
            else{
                changeSpeed(0.25, 0.25);
            }
        }
    }

    // After reaching the excavation area, go through mining
    // sequence
    if(robotState==EXCAVATE){
    }

    // After mining, return to start position
    if(robotState==GO_TO_HOME){
        if (!(position.yaw < this->destAngle+5 && position.yaw > this->destAngle-5)) {
            if(position.yaw - this->destAngle > 180 || position.yaw - this->destAngle < 0){
                changeSpeed(0.15, -0.15);
            }
            else{
                changeSpeed(-0.15, 0.15);
            }
        } 
        else{ 
            if(abs(this->position.x) > abs(this->destX)){
                changeSpeed(0.0, 0.0);
                if(this->currentPath.empty()){
                    robotState = DOCK;
                }
                else{
                    std::pair<int, int> current = this->currentPath.top();
                    this->currentPath.pop();
                    setDestZ(current.first);
                    setDestX(current.second);
                    setDestAngle(getAngle());
                }
            }
            else if(abs(this->position.x) > abs(this->destX) - 0.1){
                changeSpeed(0.1, 0.1);
            }
            else if(abs(this->position.x) > abs(this->destX) - 0.25){
                changeSpeed(0.15, 0.15);
            }
            else{
                changeSpeed(0.25, 0.25);
            }
        }
    }

    // After reaching start position, dock at dump bin
    if(robotState==DOCK){

        robotState = DUMP;
    }

    // Dump the collected rocks in the dump bin
    if(robotState==DUMP){
    }

    // After dumping the rocks, return to start position and
    // start again
    if(robotState==RETURN_TO_START){

        robotState = ALIGN;
    }

    if(robotState == OBSTACLE){
        RCLCPP_INFO(this->node->get_logger(), "position.distance: %f", position.distance);
        setStartPosition(this->search.row - std::ceil(position.z * 10), std::ceil(position.x * 10));
        int x = this->search.row - std::ceil(position.z * 10);
        int y = std::ceil(position.x * 10) + std::floor(position.distance);
        this->search.setObstacle(x, y, 2);
        aStar();
        RCLCPP_INFO(this->node->get_logger(), "Current Position: %d, %d", this->search.startX, this->search.startY);
        setGo();
        std::pair<int, int> initial = this->currentPath.top();
        this->currentPath.pop();
        setDestZ(initial.first);
        setDestX(initial.second);
        setDestAngle(getAngle());
        robotState = GO_TO_DIG_SITE;
    }
}
    

void Automation1::publishAutomationOut(){
    std::string robotStateString = robotStateMap.at(robotState);
    std::string excavationStateString = excavationStateMap.at(excavationState);
    std::string errorStateString = errorStateMap.at(errorState);
    std::string dumpStateString = dumpStateMap.at(dumpState);
    publishAutonomyOut(robotStateString, excavationStateString, errorStateString, dumpStateString);
}