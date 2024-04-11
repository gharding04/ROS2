#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <messages/msg/linear_out.hpp>
#include <messages/msg/talon_out.hpp>
#include <messages/msg/autonomy_out.hpp>

#include "AutomationTypes.hpp"
#include "search.hpp"


class Automation{
    private:
    struct MotorOut{
        float busVoltage;
        float outputCurrent;
        float outputVoltage;
        float outputPercentage;
    };
    public:

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveLeftSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveRightSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > armSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > bucketSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty_<std::allocator<void> >, std::allocator<void> > > goPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty_<std::allocator<void> >, std::allocator<void> > > stopPublisher;
    std::shared_ptr<rclcpp::Publisher<messages::msg::AutonomyOut_<std::allocator<void> >, std::allocator<void> > > autonomyOutPublisher;

    rclcpp::Node::SharedPtr node;
    Position position;
    Quaternion orientationQuaternion;
    EulerAngles orientation;
    float currentLeftSpeed=0;
    float currentRightSpeed=0;
    Linear linear1, linear2, linear3, linear4;
    MotorOut talon1, talon2, talon3, talon4, falcon1, falcon2, falcon3, falcon4;
    float destX = 0, destZ = 0, destAngle=0;
    std::stack<Coord> currentPath;
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> startBackupTime;
    Search search = Search();
    bool holes = false;

    bool runSensorlessly = false;

    virtual void automate() = 0;

    virtual void publishAutomationOut() = 0;

    void setNode(rclcpp::Node::SharedPtr node);

    void setPosition(Position position);

    void changeSpeed(float left, float right);

    void setBucketSpeed(float speed);

    void setArmSpeed(float speed);

    EulerAngles toEulerAngles(Quaternion q); 

    void setGo();

    void setStop();

    void stopActuators();

    void setLinear1(const messages::msg::LinearOut::SharedPtr linearOut);

    void setLinear2(const messages::msg::LinearOut::SharedPtr linearOut);

    void setLinear3(const messages::msg::LinearOut::SharedPtr linearOut);

    void setLinear4(const messages::msg::LinearOut::SharedPtr linearOut);

    void setTalon1(const messages::msg::TalonOut::SharedPtr talonOut);

    void setTalon2(const messages::msg::TalonOut::SharedPtr talonOut);

    void setTalon3(const messages::msg::TalonOut::SharedPtr talonOut);

    void setTalon4(const messages::msg::TalonOut::SharedPtr talonOut);

    void setFalcon1(const messages::msg::FalconOut::SharedPtr falconOut);
    
    void setFalcon2(const messages::msg::FalconOut::SharedPtr falconOut);
    
    void setFalcon3(const messages::msg::FalconOut::SharedPtr falconOut);
    
    void setFalcon4(const messages::msg::FalconOut::SharedPtr falconOut);

    bool checkErrors(Linear linear);

    void setDestAngle(float degrees);

    float getAngle();

    void setDestX(float meters);

    void setDestZ(float meters);

    void publishAutonomyOut(std::string robotStateString, std::string excavationStateString, std::string errorStateString, std::string dumpStateString);

    void setStartTime(std::chrono::time_point<std::chrono::high_resolution_clock> StartTime);

    void setBackupStartTime(std::chrono::time_point<std::chrono::high_resolution_clock> StartTime);

    std::chrono::time_point<std::chrono::high_resolution_clock> getStartTime();

    std::chrono::time_point<std::chrono::high_resolution_clock> getBackupStartTime();

    void setRunSensorlessly(bool value);

    void setStartPosition(int x, int y);

    void setDestPosition(int x, int y);

    void aStar(bool includeHoles = false);
};
