#include <cmath>
#include <ctime>

#include "logic/Automation.hpp"
#include "logic/Automation1.hpp"

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
        
    }

    if(robotState == INITIAL){
        RCLCPP_INFO(this->node->get_logger(), "Initialize");
        setDestPosition(destX, destY);
        robotState = DIAGNOSTICS;
        auto start = std::chrono::high_resolution_clock::now();
        setStartTime(start);
    }

    if(robotState==DIAGNOSTICS){
        RCLCPP_INFO(this->node->get_logger(), "Diagnostics");
        auto finish = std::chrono::high_resolution_clock::now();
        if(diagnosticsState==TALON_EXTEND){
            setBucketSpeed(1.0);
            setArmSpeed(1.0);
            if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-getStartTime()).count() > 800){
                setBucketSpeed(-1.0);
                setArmSpeed(-1.0);
                if(linear1.error != "None" && linear1.error != "PotentiometerError"){
                    RCLCPP_INFO(this->node->get_logger(), "linear1.error");
                    RCLCPP_INFO(this->node->get_logger(), linear1.error);
                    errorState = TALON_14_ERROR;
                    robotState = ROBOT_IDLE;
                }
                else if(linear2.error != "None" && linear2.error != "PotentiometerError"){
                    RCLCPP_INFO(this->node->get_logger(), "linear2.error");
                    RCLCPP_INFO(this->node->get_logger(), linear2.error);
                    errorState = TALON_15_ERROR;
                    robotState = ROBOT_IDLE;
                }
                else if(linear3.error != "None" && linear3.error != "PotentiometerError"){
                    RCLCPP_INFO(this->node->get_logger(), "linear3.error");
                    RCLCPP_INFO(this->node->get_logger(), linear3.error);
                    errorState = TALON_16_ERROR;
                    robotState = ROBOT_IDLE;
                }
                else if(linear4.error != "None" && linear4.error != "PotentiometerError"){
                    RCLCPP_INFO(this->node->get_logger(), "linear4.error");
                    RCLCPP_INFO(this->node->get_logger(), linear4.error);
                    errorState = TALON_17_ERROR;
                    robotState = ROBOT_IDLE;
                }
                else{
                    setStartTime(std::chrono::high_resolution_clock::now());
                    diagnosticsState = TALON_RETRACT;
                }
            }
        }
        if(diagnosticsState==TALON_RETRACT){
            RCLCPP_INFO(this->node->get_logger(), "Talon Retract");

            if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-getStartTime()).count() > 800){
                setBucketSpeed(-1.0);
                setArmSpeed(-1.0);
                setStartTime(std::chrono::high_resolution_clock::now());
                changeSpeed(0.05, 0.05);
                diagnosticsState = FALCON_FORWARD;
            }
        }
        if(diagnosticsState==FALCON_FORWARD){
            RCLCPP_INFO(this->node->get_logger(), "Falcon Forward");

            if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-getStartTime()).count() > 500){
                changeSpeed(0.0, 0.0);
                if(falcon1.outputCurrent == 0.0){
                    errorState = FALCON_10_ERROR;
                    robotState = ROBOT_IDLE;
                }
                else if(falcon2.outputCurrent == 0.0){
                    errorState = FALCON_11_ERROR;
                    robotState = ROBOT_IDLE;
                }
                else if(falcon3.outputCurrent == 0.0){
                    errorState = FALCON_12_ERROR;
                    robotState = ROBOT_IDLE;
                }
                else if(falcon4.outputCurrent == 0.0){
                    errorState = FALCON_13_ERROR;
                    robotState = ROBOT_IDLE;
                }
                else{
                    diagnosticsState = DIAGNOSTICS_IDLE;
                    robotState = LOCATE;
                }
            }
        }
        
    }

    // TODO: Change this to align
    if(robotState==LOCATE){
        changeSpeed(0.15,-0.15);
        if(position.arucoVisible==true){
            setDestAngle(position.yaw + 90.0);
            changeSpeed(0,0);
            robotState=ALIGN;
        }
    }

    // After finding the Aruco marker, turn the bot to 
    // align with the arena
    if(robotState==ALIGN){
        if (!(position.yaw < this->destAngle+2 && position.yaw > this->destAngle-2)) {
            changeSpeed(0.15, -0.15);
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
    std::string diagnosticsStateString = diagnosticsStateMap.at(diagnosticsState);
    publishAutonomyOut(robotStateString, excavationStateString, errorStateString, diagnosticsStateString);
}