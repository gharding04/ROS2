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
    if(robotState==ROBOT_IDLE){
        //if(deltaX < falcon1.outputPercentage * 0.05 || deltaZ < falcon1.outputPercentage * 0.05){
        //    RCLCPP_INFO(this->node->get_logger(), "ERROR: Robot not moving");
        //}

    }

    if(robotState == INITIAL){
        RCLCPP_INFO(this->node->get_logger(), "Initialize");
        setDestPosition(destX, destY);
        setBucketSpeed(1.0);
        setArmSpeed(1.0);
        auto start = std::chrono::high_resolution_clock::now();
        setStartTime(start);
        RCLCPP_INFO(this->node->get_logger(), "linear1.potentiometer: %d", linear1.potentiometer);
        RCLCPP_INFO(this->node->get_logger(), "linear3.potentiometer: %d", linear3.potentiometer);
        robotState = EXCAVATE;
        excavationState = RAISE_ARM;
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
                if(linear1.error == "ActuatorNotMovingError"){
                    RCLCPP_INFO(this->node->get_logger(), "linear1.error");
                    RCLCPP_INFO(this->node->get_logger(), "%s", linear1.error.c_str());
                    RCLCPP_INFO(this->node->get_logger(), "Talon1.maxCurrent: %f", talon1.maxCurrent);
                    errorState = TALON_14_ERROR;
                    diagnosticsState = DIAGNOSTICS_ERROR_RECOVERY;
                    robotState = ROBOT_IDLE;
                    setBucketSpeed(0.0);
                    setArmSpeed(0.0);
                }
                else if(linear2.error == "ActuatorNotMovingError"){
                    RCLCPP_INFO(this->node->get_logger(), "linear2.error");
                    RCLCPP_INFO(this->node->get_logger(), "%s", linear2.error.c_str());
                    RCLCPP_INFO(this->node->get_logger(), "Talon2.maxCurrent: %f", talon2.maxCurrent);
                    errorState = TALON_15_ERROR;
                    diagnosticsState = DIAGNOSTICS_ERROR_RECOVERY;
                    setBucketSpeed(0.0);
                    setArmSpeed(0.0);
                }
                else if(linear3.error == "ActuatorNotMovingError"){
                    RCLCPP_INFO(this->node->get_logger(), "linear3.error");
                    RCLCPP_INFO(this->node->get_logger(), "%s", linear3.error.c_str());
                    RCLCPP_INFO(this->node->get_logger(), "Talon3.maxCurrent: %f", talon3.maxCurrent);
                    errorState = TALON_16_ERROR;
                    diagnosticsState = DIAGNOSTICS_ERROR_RECOVERY;
                    setBucketSpeed(0.0);
                    setArmSpeed(0.0);
                }
                else if(linear4.error == "ActuatorNotMovingError"){
                    RCLCPP_INFO(this->node->get_logger(), "linear4.error");
                    RCLCPP_INFO(this->node->get_logger(), "%s", linear4.error.c_str());
                    RCLCPP_INFO(this->node->get_logger(), "Talon4.maxCurrent: %f", talon4.maxCurrent);
                    errorState = TALON_17_ERROR;
                    diagnosticsState = DIAGNOSTICS_ERROR_RECOVERY;
                    setBucketSpeed(0.0);
                    setArmSpeed(0.0);
                }
                else{
                    setStartTime(std::chrono::high_resolution_clock::now());
                    diagnosticsState = TALON_RETRACT;
                }
            }
        }
        if(diagnosticsState==TALON_RETRACT){
            RCLCPP_INFO(this->node->get_logger(), "Talon Retract");
            setBucketSpeed(-1.0);
            setArmSpeed(-1.0);
            if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-getStartTime()).count() > 800){
                setBucketSpeed(0.0);
                setArmSpeed(0.0);
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
                    robotState = ROBOT_IDLE;
                }
            }
        }
        if(diagnosticsState==DIAGNOSTICS_ERROR_RECOVERY){
            RCLCPP_INFO(this->node->get_logger(), "Error Recovery");
            if(errorState == TALON_14_ERROR){
                
            }
            else if(errorState == TALON_15_ERROR){
            
            }
            else if(errorState == TALON_16_ERROR){

            }
            else if(errorState == TALON_17_ERROR){

            }
            if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-getStartTime()).count() > 500){
                diagnosticsState = DIAGNOSTICS_IDLE;
                robotState = ROBOT_IDLE;
            }
        }
        
    }

    // TODO: Change this to align
    if(robotState==LOCATE){
        changeSpeed(0.15,-0.15);
        if(position.arucoVisible==true){
            RCLCPP_INFO(this->node->get_logger(), "Roll: %f Pitch: %f Yaw: %f", position.roll, position.pitch, position.yaw);
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
    // Check that the potentiometers are in the correct range
    if(robotState==EXCAVATE){
        if(excavationState == RAISE_ARM){
            if(linear1.potentiometer > 330 && linear1.potentiometer < 350){
                setArmSpeed(0.0);
            }
            if(linear3.potentiometer > 405 && linear3.potentiometer < 425){
                setBucketSpeed(0.0);
            }
            if(linear1.potentiometer > 330 && linear3.potentiometer > 405){
                changeSpeed(0.2, 0.2);
                excavationState = COLLECT;
            }
        }
        if(excavationState == COLLECT){
            if(deltaX < falcon1.outputPercentage * 0.05 || deltaZ < falcon1.outputPercentage * 0.05){
                // Raise arm by 10
                auto start = std::chrono::high_resolution_clock::now();
                setStartTime(start);
                changeSpeed(0.0, 0.0);
                setArmSpeed(1.0);
                auto finish = std::chrono::high_resolution_clock::now();
                if(if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() > 250)){
                    setArmSpeed(1.0);
                    changeSpeed(0.2, 0.2);
                }
            }
            else{
                
            }
        }
        if(excavationState == RAISE_BUCKET){

        }
        if(excavationState == LOWER_ARM){

        }
        if(excavationState == LOWER_ARM){
        }

        if(excavationState == LOWER_BUCKET){

        }
        if(excavationState == EXCAVATION_ERROR_RECOVERY){
            
        }

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
        RCLCPP_INFO(this->node->get_logger(), "linear1.potentiometer: %d", linear1.potentiometer);
        RCLCPP_INFO(this->node->get_logger(), "linear3.potentiometer: %d", linear3.potentiometer);

        if(linear1.potentiometer > 330 && linear1.potentiometer < 350){
            setArmSpeed(0.0);
        }
        if(linear3.potentiometer > 405 && linear3.potentiometer < 425){
            setBucketSpeed(0.0);
        }
        if(linear1.potentiometer > 330 && linear3.potentiometer > 405){
            robotState = DUMP;
            setBucketSpeed(1.0);
            setArmSpeed(1.0);
        }
    }

    // Dump the collected rocks in the dump bin
    if(robotState==DUMP){
        if(linear1.potentiometer > 820){
            setArmSpeed(0.0);
        }
        if(linear3.potentiometer > 850){
            setBucketSpeed(0.0);
        }
        if(linear1.potentiometer > 820 && (linear3.potentiometer > 850)){
            robotState = INITIAL;
            setBucketSpeed(-1.0);
            setArmSpeed(-1.0);
        }
    }

    // After dumping the rocks, return to start position and
    // start again
    if(robotState==RETURN_TO_START){

        robotState = ALIGN;
    }

    if(robotState == OBSTACLE){
        setStartPosition(this->search.row - std::ceil(position.z * 10), std::ceil(position.x * 10));
        int x = this->search.row - std::ceil(position.z * 10);
        int y = std::ceil(position.x * 10);
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

void Automation1::setDiagnostics(){
    robotState = DIAGNOSTICS;
    diagnosticsState = TALON_EXTEND;
    auto start = std::chrono::high_resolution_clock::now();
    setStartTime(start);
}

void Automation1::startAutonomy(){
    robotState = INITIAL;
    auto start = std::chrono::high_resolution_clock::now();
    setStartTime(start);
}
