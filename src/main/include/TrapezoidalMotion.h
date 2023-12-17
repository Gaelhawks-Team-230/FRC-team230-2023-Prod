#pragma once
#include "Common.h"

class TrapezoidalMotion
{

public:
    TrapezoidalMotion(double dist, double max_acceleration, double max_vel);
    void Service(double *cmd);
    bool CheckRoutineStatus(void) { return isRoutineDone; };

private:
    unsigned int count;
    bool isRoutineDone;

    double max_velocity;
    double acceleration;
    double deceleration;
    double current_velocity;
    double time_to_accelerate;
    double time_to_decelerate;
    double distance_to_accelerate;
    double distance_to_decelerate;
    double distance_to_travel;
    double time_to_travel_at_max_velocity;
    double total_time;
    double current_position;
};