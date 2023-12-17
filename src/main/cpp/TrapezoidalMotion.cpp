#include "TrapezoidalMotion.h"
#include <cmath>

TrapezoidalMotion::TrapezoidalMotion(double dist, double max_acceleration, double max_vel)
{
    current_velocity = 0.0;
    isRoutineDone = false;
    count = 0;

    distance_to_travel = dist;
    acceleration = max_acceleration;
    deceleration = max_acceleration;
    max_velocity = max_vel;

    time_to_accelerate = max_velocity / acceleration;
    time_to_decelerate = max_velocity / acceleration;
    distance_to_accelerate = 0.5 * acceleration * time_to_accelerate * time_to_accelerate;
    distance_to_decelerate = 0.5 * deceleration * time_to_decelerate * time_to_decelerate;

    // Calculate the motion profile
    time_to_travel_at_max_velocity =
        (fabs(distance_to_travel) - distance_to_accelerate - distance_to_decelerate) / max_velocity;
    total_time = time_to_accelerate + time_to_travel_at_max_velocity + time_to_decelerate;
}

void TrapezoidalMotion::Service(double *cmd)
{
    double time_elapsed;
    count++;
    time_elapsed = LOOPTIME * count;
    // Calculate the current position and velocity
    if (time_elapsed <= time_to_accelerate)
    {
        current_velocity += acceleration * LOOPTIME;
    }
    else if (time_elapsed <= (time_to_accelerate + time_to_travel_at_max_velocity))
    {
        current_velocity = max_velocity;
    }
    else
    {
        current_velocity -= deceleration * LOOPTIME;
    }
    current_position += current_velocity * LOOPTIME;

    // Check if the motion profile is completed
    if (time_elapsed >= total_time)
    {
        isRoutineDone = true;
    }
    else
    {
        *cmd = current_velocity*SIGN(distance_to_travel);
    }
}