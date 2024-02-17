#include <cmath>
#include <ctime>

#include "logic/Automation.hpp"
#include "logic/Automation1.hpp"
#include "logic/search.hpp"

int left = 0;

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
            destination.x=-2;
            destination.z=1;
            changeSpeed(0,0);
            robotState=ALIGN;
        }
    }
/*  
    if(robotState==GO_TO_DIG_SITE){
        double yawRadians=this->orientation.roll;

        double facingUnitX=-sin(yawRadians);
        double facingUnitZ=cos(yawRadians);
        double directionX=destination.x-position.x;
        double directionZ=destination.z-position.z;

        double theta = acos((facingUnitX*directionX + facingUnitZ*directionZ)/(sqrt(directionX*directionX + directionZ*directionZ)))*180/M_PI;
        double yaw = yawRadians * 180/M_PI;
        double deltaYaw = theta-yaw;
        double yawTolerance=5;
        if(deltaYaw > yawTolerance){
            changeSpeed(-0.15,0.15);
        }
        else if (deltaYaw < yawTolerance){
            changeSpeed(0.15,-0.15);
        }
        else{
            changeSpeed(0.15 - 0.1*deltaYaw/yawTolerance,0.15 + 0.1*deltaYaw/yawTolerance);
        }
        std::cout << orientation.roll*180/M_PI << ", " << orientation.pitch*180/M_PI << ", " << orientation.yaw*180/ M_PI << "   "
                << "   \t" << position.x << "  " << position.y << "  " << position.z
                << "   \t" << position.ox << "  " << position.oy << "  " << position.oz << "  " << position.ow
                << "   \t" << facingUnitX << " " << facingUnitZ << "   " << yaw << " " << deltaYaw << " " << theta
              << "   \t" << position.arucoVisible << std::endl;
    }
*/    

    // After finding the Aruco marker, turn the bot to 
    // align with the arena
    if(robotState==ALIGN){
        RCLCPP_INFO(this->node->get_logger(), "Left: %d", left);
        if (!(position.yaw < this->destAngle+5 && position.yaw > this->destAngle-5)) {
            changeSpeed(0.15*left, -0.15*left);
        } else {
            changeSpeed(0, 0);
            setDestDistance(position.x - 1.0);
            setGo();
            changeSpeed(0.25, 0.25);
            robotState = GO_TO_DIG_SITE;
        }
    }

    // After aligning with the arena, navigate to the 
    // excavation area
    if(robotState==GO_TO_DIG_SITE){
        RCLCPP_INFO(this->node->get_logger(), "GO_TO_DIG_SITE");
        RCLCPP_INFO(this->node->get_logger(), "ZedPosition.z: %f", this->position.z);
        RCLCPP_INFO(this->node->get_logger(), "Left: %d", left);
        if(abs(this->position.x) > abs(this->destDistance)){
            changeSpeed(0.0, 0.0);
            robotState = EXCAVATE;
        }
        else if(abs(this->position.x) > abs(this->destDistance) - 0.1){
            changeSpeed(0.1, 0.1);
        }
        else if(abs(this->position.x) > abs(this->destDistance) - 0.25){
            changeSpeed(0.15, 0.15);
        }
        else{
            changeSpeed(0.25, 0.25);
        }
    }

    // After reaching the excavation area, go through mining
    // sequence
    if(robotState==EXCAVATE){
    }

    // After mining, return to start position
    if(robotState==GO_TO_HOME){
        if (!(position.yaw < this->destAngle+5 && position.yaw > this->destAngle-5)) {
            changeSpeed(0.15, -0.15);
        }
        else if(abs(this->position.x) > abs(this->destDistance)){
            changeSpeed(0.0, 0.0);
            robotState = DOCK;
        }
        else if(abs(this->position.x) > abs(this->destDistance) - 0.1){
            changeSpeed(0.1, 0.1);
        }
        else if(abs(this->position.x) > abs(this->destDistance) - 0.25){
            changeSpeed(0.15, 0.15);
        }
        else{
            changeSpeed(0.25, 0.25);
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
}
    

void Automation1::publishAutomationOut(){
    std::string robotStateString = robotStateMap.at(robotState);
    std::string excavationStateString = excavationStateMap.at(excavationState);
    std::string errorStateString = errorStateMap.at(errorState);
    std::string dumpStateString = dumpStateMap.at(dumpState);
    publishAutonomyOut(robotStateString, excavationStateString, errorStateString, dumpStateString);
}