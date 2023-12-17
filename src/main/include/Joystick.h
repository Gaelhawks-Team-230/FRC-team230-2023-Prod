#pragma once
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>

class TalonXXV;

#define MAX_JOYSTICK_BUTTONS 25
#define MAX_JOYSTICK_AXES 8

enum ButtonState
{
    kOff = 0,
    kPressing = 1,
    kHeld = 2,
    kReleasing = 3,
};
enum Gampad_POV
{
    Up = 0,
    Right = 90,
    DIAG_DOWN_RIGHT=135,
    Down = 180,
    DIAG_DOWN_LEFT=225,
    Left = 270
};
enum ThreeWaySwitch
{
    Bottom = 0,
    Middle = 1,
    Top = 2,
};

class Joystick
{
private:
    TalonXXV *mainRobot;
    frc::Joystick *flightctrl;
    frc::Joystick *gamepad;

    ButtonState m_flightctrlBtnState[MAX_JOYSTICK_BUTTONS];
    ButtonState m_gamepadBtnState[MAX_JOYSTICK_BUTTONS];

    int m_flightctrlBtnCount;
    int m_flightctrlAxisCount;
    int m_gamepadBtnCount;
    int m_gamepadAxisCount;
    int m_gamepadPOVCount;

    double m_gamepadDeadband_x;
    double m_gamepadShaping_x;
    double m_gamepadDeadband_y;
    double m_gamepadShaping_y;
    double m_gamepadDeadband_r;
    double m_gamepadShaping_r;

    double m_flightctrlDeadband_x;
    double m_flightctrlShaping_x;
    double m_flightctrlDeadband_y;
    double m_flightctrlShaping_y;
    double m_flightctrlDeadband_r;
    double m_flightctrlShaping_r;

    double flightctrl_raw_x;
    double flightctrl_raw_y;
    double flightctrl_raw_z;
    double flightctrl_raw_r;
    double flightctrl_shape_x;
    double flightctrl_shape_y;
    double flightctrl_shape_r;
    double flightctrl_scale_x;
    double flightctrl_scale_y;
    double flightctrl_scale_r;
    double flightctrl_cmd_x;
    double flightctrl_cmd_y;
    double flightctrl_cmd_r;
    double flightctrl_x;
    double flightctrl_y;
    double flightctrl_r;
    double flightctrl_z;

    double gamepad_raw_x;
    double gamepad_raw_y;
    double gamepad_raw_z;
    double gamepad_raw_r;
    double gamepad_shape_x;
    double gamepad_shape_y;
    double gamepad_shape_r;
    double gamepad_scale_x;
    double gamepad_scale_y;
    double gamepad_scale_r;
    double gamepad_x;
    double gamepad_y;
    double gamepad_z;
    double gamepad_r;

    double xf;
    double yf;
    double zf;

    ButtonState m_DpadUp;
    ButtonState m_DpadRight;
    ButtonState m_DpadDown;
    ButtonState m_DpadLeft;

    ButtonState BtnInputLoop(bool curPressed, ButtonState curState);
    double AxisShaping(double rawReading, double deadband, double shapingSlope);
    double ScaleCmd(double shapedReading, double axis_2_vel);

public:
    Joystick(TalonXXV *pRobot);

    void JoystickCountInitialize(void);
    void LocalReset(void);
    void StopAll(void);
    void UpdateDash(void);
    void Analyze(void);
    void CmdModel(double x, double y, double z,
                  double *pxout, double *pyout, double *pzout,
                  double kx, double ky, double kz);
    ThreeWaySwitch GetThreeWaySwitch(int btnNumber1, int btnNumber2);

    ButtonState GetGamepadButton(int btnNumber);
    ButtonState GetFlightCtrlButton(int btnNumber);
    bool GamepadBtnPushed(int btnNumber);
    bool FlightCtrlBtnPushed(int btnNumber);

    ButtonState GetDpadUpButton() { return m_DpadUp; };
    ButtonState GetDpadRightButton() { return m_DpadRight; };
    ButtonState GetDpadDownButton() { return m_DpadDown; };
    ButtonState GetDpadLeftButton() { return m_DpadLeft; };

    bool GetDpadUpPushed();
    bool GetDpadRightPushed();
    bool GetDpadDownPushed();
    bool GetDpadLeftPushed();

    double GetFlightCtrl_CMD_X() { return flightctrl_cmd_x; };
    double GetFlightCtrl_CMD_Y() { return flightctrl_cmd_y; };
    double GetFlightCtrl_CMD_R() { return flightctrl_cmd_r; };

    double GetFlightCtrl_RAW_Z() { return flightctrl_raw_z; };

    double GetGamepad_RAW_X() {return gamepad_raw_x;};
    double GetGamepad_RAW_Y() {return gamepad_raw_y;};
    double GetGamepad_RAW_R() {return gamepad_raw_r;};
    // double GetGamepad_y() { return gamepad_y; };
    // double GetGamepad_z() { return gamepad_z; };
};