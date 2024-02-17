#pragma once

#include "Automation.hpp"


class Automation1 : public Automation{

    enum RobotState{LOCATE,ALIGN,GO_TO_DIG_SITE,EXCAVATE,GO_TO_HOME,DOCK,DUMP,RETURN_TO_START,ROBOT_IDLE};
    RobotState robotState=EXCAVATE;
    Location destination;

    std::map<RobotState, const char*> robotStateMap = {
        {LOCATE, "Locate"},
        {ALIGN, "Align"},
        {GO_TO_DIG_SITE, "Go To Dig Site"},
        {EXCAVATE, "Excavate"},
        {GO_TO_HOME, "Go To Home"},
        {DOCK, "Dock"},
        {DUMP, "Dump"},
        {RETURN_TO_START, "Return To Start"},
        {ROBOT_IDLE,  "Idle"}
    };

    void automate();

    void publishAutomationOut();

};
