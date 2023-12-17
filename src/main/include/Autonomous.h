#pragma once

#include "Common.h"
#include "TrapezoidalMotion.h"

//  Needs to be calibrated in real practice match
#define HORZ_VEL_TO_CUBE (26.6) // practice field dist
#define PATH_H_SPEED_SF (0.5)
#define PATH_H_TIME_S (1.0/PATH_H_SPEED_SF)

#define PATH_H_SPEED_SF_SLOW (0.5)
#define PATH_H_TIME_S_SLOW (1.0/PATH_H_SPEED_SF_SLOW)


class TalonXXV;
class Autonomous
{
public:
    Autonomous(TalonXXV *pRobot);
    void LocalReset(void);
    void StartingConfig(void);
    void PathA(); // path cone balance via charge
    void PathB(); // path place cone
    void PathC(); // path place cone and exit
    void PathD(); // path cone and cube
    void PathE(); // path cone, cube and balance via charge
    void PathF(); // path cone, cube, cube
    void PathG(); // path cone and cube on dirty side
    void PathH(); // path dirty side 2.5 gamepiece
    void PathI(); // path nosemode cone, cube, and balance
    void WaitForTime(double time);

    // path parts
    double GetAutoXDot(void) { return xdot; };
    double GetAutoYDot(void) { return ydot; };
    double GetAutoPsidot(void) { return psidot; };
    void StopDrive(void)
    {
        xdot = 0.0;
        ydot = 0.0;
        psidot = 0.0;
    };

    void AccelProfile(double time, double step);
    void ConstProfile(double time, double vel);

    void ConstProfileDynamic(double time, double xVel, double yVel, double psiVel);
    void AccelProfileDynamic(double time, double xStep, double yStep, double psiStep);


    void TrajAccelProfile(double stageTime, double vxStart, double vyStart, double vpsiStart, double vxStop, double vyStop, double vpsiStop);
    void TrajConstProfile(double stageTime, double vx, double vy, double vpsi);

private:
    TalonXXV *mainRobot;

    unsigned int autoStage;
    // local stages
    unsigned int countOffset;
    double elaspedTime;

    TrapezoidalMotion *prepareToScore;
    TrapezoidalMotion *moveToScore;
    TrapezoidalMotion *exitCommunity;
    TrapezoidalMotion *ejectGamepiece;
    TrapezoidalMotion *prepareToBalance;



    // TrapezoidalMotion *doubleConeX;
    // TrapezoidalMotion *doubleConeY;

    double xdot, ydot, psidot;
};