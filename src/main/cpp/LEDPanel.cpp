#include "LEDPanel.h"

LEDPanel::LEDPanel(TalonXXV *pRobot)
{
    mainRobot = pRobot;

    light0 = new frc::DigitalOutput(LED_LIGHT_OUTPUT_0);
    light1 = new frc::DigitalOutput(LED_LIGHT_OUTPUT_1);
    light2 = new frc::DigitalOutput(LED_LIGHT_OUTPUT_2);

    StartingConfig();
}
void LEDPanel::LocalReset()
{
    DisplayBlank();
}
void LEDPanel::StopAll()
{
    LocalReset();
}
void LEDPanel::StartingConfig()
{
    LocalReset();
}
/**
 * @brief Default state of the light
 *
 */
void LEDPanel::DisplayBlank()
{
    light0Status = true;
    light1Status = false;
    light2Status = true;
}
/**
 * @brief Display cone to the human player
 *
 */
void LEDPanel::DisplayCone()
{
    light0Status = true;
    light1Status = false;
    light2Status = false;
}
/**
 * @brief Display cube to the human player
 *
 */
void LEDPanel::DisplayCube()
{
    light0Status = false;
    light1Status = true;
    light2Status = false;
}
/**
 * @brief Display driver station disconnected
 * 
 */
void LEDPanel::DisplayDisconnected()
{
    light0Status = true;
    light1Status = true;
    light2Status = false;
}
/**
 * @brief Display driver station connected
 * 
 */
void LEDPanel::DisplayConnected()
{
    light0Status = false;
    light1Status = false;
    light2Status = true;
}

// void LEDPanel::DisplayFMSConnected()
// {
//     light0Status = true;
//     light1Status = false;
//     light2Status = true;
// }

// void LEDPanel::DisplayBalancingPos()
// {
//     light0Status = true;
//     light1Status = true;
//     light2Status = false;
// }

// void LEDPanel::DisplayBalancingNeg()
// {
//     light0Status = false;
//     light1Status = false;
//     light2Status = true;
// }

// void LEDPanel::DisplayBalanced()
// {
//     light0Status = true;
//     light1Status = false;
//     light2Status = true;
// }

void LEDPanel::Service()
{
    light0->Set(light0Status);
    light1->Set(light1Status);
    light2->Set(light2Status);
}