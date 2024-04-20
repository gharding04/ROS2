#pragma once

struct Position{
    double x,y,z,ox,oy,oz,ow,arucoVisible,roll,pitch,yaw,aruco_roll,aruco_pitch,aruco_yaw,x_acc, y_acc, z_acc, x_vel, y_vel, z_vel;
};

struct Location{
    double x,z;
};

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

struct Linear {
    float speed = 0.0;
    std::string error = "None";
    bool atMin = false;
    bool atMax = false;
    float distance = 0.0;
    int potentiometer = 0;
    bool sensorless = false;
};

struct MotorOut{
    float busVoltage;
    float outputCurrent;
    float outputVoltage;
    float outputPercentage;
    float maxCurrent = 0.0;
};