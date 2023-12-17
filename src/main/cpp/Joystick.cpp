#include <frc/smartdashboard/SmartDashboard.h>
#include "stdio.h"
#include <cmath>
#include "Joystick.h"
#include "Common.h"
#include "TalonXXV.h"

/**
 * @brief Construct a new Joystick::Joystick object. Sets up intial values and links controllers.
 *
 * @param probot
 */
Joystick::Joystick(TalonXXV *probot)
{
    mainRobot = probot;
    flightctrl = new frc::Joystick(0); // change back to 0 after test KP
    gamepad = new frc::Joystick(1);    // change back to 1 after test KP

    m_flightctrlBtnCount = 0;
    m_gamepadBtnCount = 0;
    m_flightctrlAxisCount = 0;
    m_gamepadBtnCount = 0;

    JoystickCountInitialize();

    LocalReset();
}

/**
 * @brief Sets the inital values for the button and axis counts
 *
 */
void Joystick::JoystickCountInitialize()
{
    m_gamepadBtnCount = gamepad->GetButtonCount();
    if (m_gamepadBtnCount > MAX_JOYSTICK_BUTTONS)
    {
        printf("Gamepad has too many buttons! (read %d, clipped to %d)\n", m_gamepadBtnCount, MAX_JOYSTICK_BUTTONS);
        m_gamepadBtnCount = MAX_JOYSTICK_BUTTONS;
    }
    m_flightctrlBtnCount = flightctrl->GetButtonCount();
    if (m_flightctrlBtnCount > MAX_JOYSTICK_BUTTONS)
    {
        printf("Flight Controller has too many buttons!(read %d, clipped to %d)\n", m_flightctrlBtnCount, MAX_JOYSTICK_BUTTONS);
        m_flightctrlBtnCount = MAX_JOYSTICK_BUTTONS;
    }
    m_gamepadAxisCount = gamepad->GetAxisCount();
    if (m_gamepadAxisCount > MAX_JOYSTICK_AXES)
    {
        printf("Gamepad has too many axes! (read %d, clipped to %d)\n", m_gamepadAxisCount, MAX_JOYSTICK_AXES);
        m_gamepadAxisCount = MAX_JOYSTICK_AXES;
    }
    m_flightctrlAxisCount = flightctrl->GetAxisCount();
    if (m_flightctrlAxisCount > MAX_JOYSTICK_AXES)
    {
        printf("Flight Controller has too many axes! (read %d, clipped to %d)\n", m_flightctrlAxisCount, MAX_JOYSTICK_AXES);
        m_flightctrlAxisCount = MAX_JOYSTICK_AXES;
    }
}

/**
 * @brief Resets all the buttons to off
 *
 */
void Joystick::LocalReset()
{
    int l_btn;
    for (l_btn = 0; l_btn < MAX_JOYSTICK_BUTTONS; l_btn++)
    {
        m_gamepadBtnState[l_btn] = kOff;
    }
    for (l_btn = 0; l_btn < MAX_JOYSTICK_BUTTONS; l_btn++)
    {
        m_flightctrlBtnState[l_btn] = kOff;
    }

    m_DpadUp = kOff;
    m_DpadRight = kOff;
    m_DpadDown = kOff;
    m_DpadLeft = kOff;

    flightctrl_raw_x = 0.0;
    flightctrl_raw_y = 0.0;
    flightctrl_raw_z = 0.0;
    flightctrl_raw_r = 0.0;

    flightctrl_shape_x = 0.0;
    flightctrl_shape_y = 0.0;
    flightctrl_shape_r = 0.0;

    flightctrl_scale_x = 0.0;
    flightctrl_scale_y = 0.0;
    flightctrl_scale_r = 0.0;

    flightctrl_cmd_x = 0.0;
    flightctrl_cmd_y = 0.0;
    flightctrl_cmd_r = 0.0;

    gamepad_raw_x = 0.0;
    gamepad_raw_y = 0.0;
    gamepad_raw_z = 0.0;
    gamepad_raw_r = 0.0;

    gamepad_shape_x = 0.0;
    gamepad_shape_y = 0.0;
    gamepad_shape_r = 0.0;

    gamepad_scale_x = 0.0;
    gamepad_scale_y = 0.0;
    gamepad_scale_r = 0.0;

    xf = 0.0;
    yf = 0.0;
    zf = 0.0;
}

/**
 * @brief just another name for the reset
 *
 */
void Joystick::StopAll()
{
    LocalReset();
}

/**
 * @brief sends the data that you designate to the dashboard
 *
 */
void Joystick::UpdateDash()
{
    // tbd based on what info is needed on dash
}

/**
 * @brief returns the button state of each button
 *
 * @param btnNumber
 * @return ButtonState
 */
ButtonState Joystick::GetGamepadButton(int btnNumber)
{
    if ((btnNumber <= 0) || (btnNumber > m_gamepadBtnCount))
    {
        return (kOff);
    }
    else
    {
        return (m_gamepadBtnState[btnNumber - 1]);
    }
}

/**
 * @brief returns the button state of each button
 *
 * @param btnNumber
 * @return ButtonState
 */
ButtonState Joystick::GetFlightCtrlButton(int btnNumber)
{
    if ((btnNumber <= 0) || (btnNumber > m_flightctrlBtnCount))
    {
        return (kOff);
    }
    else
    {
        return (m_flightctrlBtnState[btnNumber - 1]);
    }
}

/**
 * @brief determines whether the gamepad button is being pushed or not based on the enum button state
 *
 * @param btnNumber
 * @return true
 * @return false
 */
bool Joystick::GamepadBtnPushed(int btnNumber)
{
    if ((btnNumber <= 0) || (btnNumber > m_gamepadBtnCount))
    {
        return false;
    }
    if ((m_gamepadBtnState[btnNumber - 1] == kPressing) || (m_gamepadBtnState[btnNumber - 1] == kHeld))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief determines whether the flight controller button is being pushed or not based on the enum button state
 *
 * @param btnNumber
 * @return true
 * @return false
 */
bool Joystick::FlightCtrlBtnPushed(int btnNumber)
{
    if ((btnNumber <= 0) || (btnNumber > m_flightctrlBtnCount))
    {
        return (false);
    }
    if (m_flightctrlBtnState[btnNumber - 1] == kPressing || m_flightctrlBtnState[btnNumber - 1] == kHeld)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief the function returns true/false based on if the dpad button is either "held"/"pressed" or not
 *
 * @return true
 * @return false
 */
bool Joystick::GetDpadUpPushed()
{
    return m_DpadUp == kHeld || m_DpadUp == kPressing;
}

/**
 * @brief the function returns true/false based on if the dpad button is either "held"/"pressed" or not
 *
 * @return true
 * @return false
 */
bool Joystick::GetDpadRightPushed()
{
    return m_DpadRight == kHeld || m_DpadRight == kPressing;
}

/**
 * @brief the function returns true/false based on if the dpad button is either "held"/"pressed" or not
 *
 * @return true
 * @return false
 */
bool Joystick::GetDpadDownPushed()
{
    return m_DpadDown == kHeld || m_DpadDown == kPressing;
}

/**
 * @brief the function returns true/false based on if the dpad button is either "held"/"pressed" or not
 *
 * @return true
 * @return false
 */
bool Joystick::GetDpadLeftPushed()
{
    return m_DpadLeft == kHeld || m_DpadLeft == kPressing;
}

/**
 * @brief The "ButtonState" of each button on both the flight controller and gamepad is defined here.
 * It operates on whether the button is pressed or not and gives you the cases upon each scenario and
 * the previous "ButtonState".
 * @param curPressed
 * @param curState
 * @return ButtonState
 */
ButtonState Joystick::BtnInputLoop(bool curPressed, ButtonState curState)
{
    ButtonState l_newState;
    l_newState = curState;

    if (curPressed)
    {
        switch (curState)
        {
        case kOff:
            l_newState = kPressing;
            break;

        case kPressing:
            l_newState = kHeld;
            break;
        case kHeld:
            l_newState = kHeld;
            break;
        case kReleasing:
            l_newState = kPressing;
            break;

        default:
            printf("Error in BtnInputLoop switch!\n");
            break;
        }
    }

    else
    {
        switch (curState)
        {
        case kOff:
            l_newState = kOff;
            break;

        case kPressing:
            l_newState = kReleasing;
            break;
        case kHeld:
            l_newState = kReleasing;
            break;
        case kReleasing:
            l_newState = kOff;
            break;

        default:
            printf("Error in BtnInputLoop switch!\n");
            break;
        }
    }
    return (l_newState);
}

/**
 * @brief returns different button states depending on the three way switch orientation
 *
 * @param btnNumber1
 * @param btnNumber2
 * @return ThreeWaySwitch
 */
ThreeWaySwitch Joystick::GetThreeWaySwitch(int btnNumber1, int btnNumber2)
{
    if (m_flightctrlBtnState[btnNumber1] && !m_flightctrlBtnState[btnNumber2])
        return Top;
    if (!m_flightctrlBtnState[btnNumber1] && m_flightctrlBtnState[btnNumber2])
        return Bottom;
    if (!m_flightctrlBtnState[btnNumber1] && !m_flightctrlBtnState[btnNumber2])
        return Middle;
    return Bottom;
}

/**
 * @brief retruns the scaled velocity command after taking the shaped axis value
 *
 * @param shapedReading
 * @param axis_2_vel
 * @return double
 */
double Joystick::ScaleCmd(double shapedReading, double axis_2_vel)
{
    return shapedReading * axis_2_vel;
}

/**
 * @brief Command filtering for the joystick axis
 *
 * @param x x cmd
 * @param y y cmd
 * @param z z cmd
 * @param pxout out
 * @param pyout out
 * @param pzout out
 * @param kx x decay
 * @param ky y decay
 * @param kz z decay
 */
void Joystick::CmdModel(double x, double y, double z,
                        double *pxout, double *pyout, double *pzout,
                        double kx, double ky, double kz)
{
    double xfdot, yfdot, zfdot;

    xfdot = (x - xf) * kx;
    yfdot = (y - yf) * ky;
    zfdot = (z - zf) * kz;

    xf = xf + xfdot * LOOPTIME;
    yf = yf + yfdot * LOOPTIME;
    zf = zf + zfdot * LOOPTIME;

    *pxout = xf;
    *pyout = yf;
    *pzout = zf;
}

/**
 * @brief This functions modifies the raw inputs returned from the axis. The deadband is a range which
 * within all inputs are null. When outside the deadband, the raw inputs are modified to a new value
 * so that the joystick is more sensitive as it gets closer to the maximum raw value. This is controlled
 * in both the negative and positive coordinate cases. This functionality is detailed in the math
 * equations listed below.
 *
 * @param rawReading
 * @param deadband
 * @param shapingSlope
 * @return double
 */
double Joystick::AxisShaping(double rawReading, double deadband, double shapingSlope)
{
    double l_modifiedReading;
    l_modifiedReading = fmax(((fabs(rawReading) - deadband) / (1.0 - deadband)), 0.0) * SIGN(rawReading);
    l_modifiedReading = ((0.5 * shapingSlope * fabs(l_modifiedReading) - 0.5 * shapingSlope + 1) * l_modifiedReading);
    return (l_modifiedReading);
}

/**
 * @brief This function is called every loop to gather and modify
 * the values needed for the flight controller and gamepad
 *
 */
void Joystick::Analyze()
{
    if (m_gamepadBtnCount == 0 || m_flightctrlBtnCount == 0)
    {
        JoystickCountInitialize();
    }

    // Convert to std frame of reference
    flightctrl_raw_x = flightctrl->GetRawAxis(1) * -1.0;
    flightctrl_raw_y = flightctrl->GetRawAxis(0);
    flightctrl_raw_z = flightctrl->GetRawAxis(2);
    flightctrl_raw_r = flightctrl->GetRawAxis(5);

    gamepad_raw_x = gamepad->GetRawAxis(0);
    // Convert to std frame of reference
    gamepad_raw_y = gamepad->GetRawAxis(1);
    gamepad_raw_z = gamepad->GetRawAxis(2);
    gamepad_raw_r = gamepad->GetRawAxis(3);

    flightctrl_shape_x = AxisShaping(flightctrl_raw_x, FC_X_DEADBAND, FC_X_SHAPING);
    // Convert to std frame of reference
    flightctrl_shape_y = AxisShaping(flightctrl_raw_y, FC_Y_DEADBAND, FC_Y_SHAPING);
    flightctrl_shape_r = AxisShaping(flightctrl_raw_r, FC_R_DEADBAND, FC_R_SHAPING);

    double k;
    k = (1.0 - GetFlightCtrl_RAW_Z()) / 2.0;

    // flightctrl_scale_x = ScaleCmd(flightctrl_shape_x, TalonXXV::Limit(XY_SFACTOR_LOW, XY_SFACTOR, (1.0 / 0.4) * (XY_SFACTOR - XY_SFACTOR_LOW) * k));
    // flightctrl_scale_y = ScaleCmd(flightctrl_shape_y, TalonXXV::Limit(XY_SFACTOR_LOW, XY_SFACTOR, (1.0 / 0.4) * (XY_SFACTOR - XY_SFACTOR_LOW) * k));
    // flightctrl_scale_r = ScaleCmd(flightctrl_shape_r, TalonXXV::Limit(R_SFACTOR_LOW, R_SFACTOR, (1.0 / 0.4) * (R_SFACTOR - R_SFACTOR_LOW) * k));
    flightctrl_scale_x = ScaleCmd(flightctrl_shape_x, TalonXXV::Limit(XY_SFACTOR_LOW, XY_SFACTOR, XY_SFACTOR_LOW + (XY_SFACTOR - XY_SFACTOR_LOW) * k));
    flightctrl_scale_y = ScaleCmd(flightctrl_shape_y, TalonXXV::Limit(XY_SFACTOR_LOW, XY_SFACTOR, XY_SFACTOR_LOW + (XY_SFACTOR - XY_SFACTOR_LOW) * k));
    flightctrl_scale_r = ScaleCmd(flightctrl_shape_r, TalonXXV::Limit(R_SFACTOR_LOW, R_SFACTOR, R_SFACTOR_LOW + (R_SFACTOR - R_SFACTOR_LOW) * k));

    CmdModel(flightctrl_scale_x, flightctrl_scale_y, flightctrl_scale_r, &flightctrl_cmd_x, &flightctrl_cmd_y, &flightctrl_cmd_r, 50.0, 50.0, 50.0);

    int l_btn;
    for (l_btn = 0; l_btn < m_gamepadBtnCount; l_btn++)
    {
        m_gamepadBtnState[l_btn] = BtnInputLoop(gamepad->GetRawButton(l_btn + 1), m_gamepadBtnState[l_btn]);
    }

    for (l_btn = 0; l_btn < m_flightctrlBtnCount; l_btn++)
    {
        m_flightctrlBtnState[l_btn] = BtnInputLoop(flightctrl->GetRawButton(l_btn + 1), m_flightctrlBtnState[l_btn]);
    }

    m_gamepadPOVCount = gamepad->GetPOVCount();
    if (m_gamepadPOVCount >= 1)
    {
        int l_angle = gamepad->GetPOV();
        m_DpadUp = BtnInputLoop(l_angle == Up, m_DpadUp);
        m_DpadRight = BtnInputLoop(l_angle == Right, m_DpadRight);
        m_DpadDown = BtnInputLoop((l_angle >= DIAG_DOWN_RIGHT && l_angle <= DIAG_DOWN_LEFT), m_DpadDown);
        m_DpadLeft = BtnInputLoop(l_angle == Left, m_DpadLeft);
    }
}