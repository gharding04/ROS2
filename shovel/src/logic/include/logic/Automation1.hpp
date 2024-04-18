#pragma once

#include "Automation.hpp"


class Automation1 : public Automation{

    enum RobotState{INITIAL,DIAGNOSTICS,LOCATE,ALIGN,GO_TO_DIG_SITE,EXCAVATE,OBSTACLE,GO_TO_HOME,DOCK,DUMP,RETURN_TO_START,ROBOT_IDLE};
    enum ExcavationState{EXCAVATION_IDLE,ERROR_RECOVERY};
    enum ErrorState {TALON_14_ERROR, TALON_15_ERROR, TALON_16_ERROR, TALON_17_ERROR, FALCON_10_ERROR, FALCON_11_ERROR, FALCON_12_ERROR, FALCON_13_ERROR,
        LOWER_ASSEMBLY_ERROR,LOWER_LADDER_ERROR,DIG_ERROR,RAISE_LADDER_ERROR,RAISE_ASSEMBLY_ERROR,RAISE_BIN_ERROR,LOWER_BIN_ERROR,NONE};
    enum DiagnosticsState{DIAGNOSTICS_IDLE,TALON_EXTEND,TALON_RETRACT,FALCON_FORWARD};
    RobotState robotState=INITIAL;
    RobotState previousState = ROBOT_IDLE;
    ExcavationState excavationState = EXCAVATION_IDLE;
    ErrorState errorState = NONE;
    DiagnosticsState diagnosticsState = TALON_EXTEND;
    Location destination;
    float normalDistance = 1.2;

    std::map<RobotState, const char*> robotStateMap = {
        {DIAGNOSTICS, "Diagnostics"},
        {LOCATE, "Locate"},
        {ALIGN, "Align"},
        {GO_TO_DIG_SITE, "Go To Dig Site"},
        {EXCAVATE, "Excavate"},
        {OBSTACLE, "Obstacle"},
        {GO_TO_HOME, "Go To Home"},
        {DOCK, "Dock"},
        {DUMP, "Dump"},
        {RETURN_TO_START, "Return To Start"},
        {ROBOT_IDLE,  "Idle"}
    };

    std::map<ExcavationState, const char*> excavationStateMap = {
        {EXCAVATION_IDLE, "Idle"},
        {ERROR_RECOVERY, "Error Recovery"}
    };

    std::map<ErrorState, const char*> errorStateMap = {
        {TALON_14_ERROR, "Talon 14 Error"},
        {TALON_15_ERROR, "Talon 15 Error"},
        {TALON_16_ERROR, "Talon 16 Error"},
        {TALON_17_ERROR, "Talon 17 Error"},
        {FALCON_10_ERROR, "Falcon 10 Error"},
        {FALCON_11_ERROR, "Falcon 11 Error"},
        {FALCON_12_ERROR, "Falcon 12 Error"},
        {FALCON_13_ERROR, "Falcon 13 Error"},
        {LOWER_ASSEMBLY_ERROR, "Lower Assembly Error"},
        {LOWER_LADDER_ERROR, "Lower Ladder Error"},
        {DIG_ERROR, "Dig Error"},
        {RAISE_LADDER_ERROR, "Raise Ladder Error"},
        {RAISE_ASSEMBLY_ERROR, "Raise Assembly Error"},
        {RAISE_BIN_ERROR, "Raise Bin Error"},
        {LOWER_BIN_ERROR, "Lower Bin Error"},
        {NONE, "None"}
    };

    std::map<DiagnosticsState, const char*> diagnosticsStateMap = {
        {DIAGNOSTICS_IDLE, "Diagnostics Idle"},
        {TALON_EXTEND, "Talon Extend"},
        {TALON_RETRACT, "Talon Retract"},
        {FALCON_FORWARD, "Falcon Forward"}
    };

    void automate();

    void publishAutomationOut();

    void setDiagnostics();

    void startAutonomy();
};
