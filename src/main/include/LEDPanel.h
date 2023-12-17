#pragma once

#include <frc/DigitalOutput.h>
#include "Common.h"

class TalonXXV;
class LEDPanel
{
public:
    LEDPanel(TalonXXV *pRobot);
    void LocalReset(void);
    void StopAll(void);
    void StartingConfig(void);
    void Service(void);

    void DisplayCone(void);
    void DisplayCube(void);
    void DisplayBlank(void);

    void DisplayDisconnected();
    void DisplayConnected();
    // void DisplayDisabled(); //DisplayDSConnected


    // void DisplayBalancingPos(void);
    // void DisplayBalancingNeg(void);
    // void DisplayBalanced(void);

private:
    TalonXXV *mainRobot;

    frc::DigitalOutput *light0;
    frc::DigitalOutput *light1;
    frc::DigitalOutput *light2;

    bool light0Status;
    bool light1Status;
    bool light2Status;
};