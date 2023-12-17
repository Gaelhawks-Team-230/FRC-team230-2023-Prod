#pragma once

#include "Common.h"
#include "frc/motorcontrol/PWMTalonSRX.h"
#include "frc/DoubleSolenoid.h"
#include "frc/PneumaticsModuleType.h"
#include "frc/DigitalInput.h"

class TalonXXV;

// control laws - wrist
#define WRIST_K_POS (6.0)
#define WRIST_K_VEL (2.0)
#define WRIST_TAU_ROBOT (0.1)
#define WRIST_K_ROBOT (120.0)

// decrease offset by 20 deg to bump wrist up
#define WRIST_ENCODER_OFFSET (182.24)

// roller cmds
#define CONE_INTAKE_ROLLER_CMD (1.0)
#define CONE_INTAKE_HOLD_CMD (0.2)
#define CONE_EJECT_ROLLER_CMD (-0.5)
#define CONE_EJECT_ROLLER_FULL_CMD (-1.0)

#define CUBE_INTAKE_ROLLER_CMD (0.75)
#define CUBE_INTAKE_HOLD_CMD (0.1)
#define CUBE_EJECT_ROLLER_CMD (-0.6)
#define CUBE_EJECT_ROLLER_FULL_CMD (-0.8)
// brought down from full cmd to 0.8 

#define GRABBER_STALL (20.0) // amps


class Grabber
{
private:
    TalonXXV *mainRobot;

    frc::PWMTalonSRX *wristMotor;
    frc::PWMTalonSRX *rollerMotor;

    frc::DoubleSolenoid *wristCylinder;
    frc::DoubleSolenoid::Value wristCylinderPos;

    frc::DigitalInput *wristEncoderInput;
    frc::DutyCycle *wristEncoder;

    frc::DigitalInput *intakeBeamBreak;

    // extender control vars
    double wristAngleCmd;
    double wristPosHat;
    double wristPosErr;
    double wristVelHat;
    double wristVelCmd;
    double wristVelErr;
    double wristAcc;
    double wristMotorCmd;
    double wristErrInt;

    double lastWristTheta;
    double wristTheta;
    double rawWristTheta;
    double wristVel;
    double wristMotorVel;

    int intakeState;
    double rollerCmd;
    double rollerCurrent;
    bool isBeamBroken;
    int beamBreakLoopcount;
    bool isGamepieceLoaded;
    bool isStalled;
    bool holdGamepiece;
    double rollerCurrentF;

    double wristAngleBias;

    Gamepiece gamepiece;

public:
    Grabber(TalonXXV *pRobot);

    void LocalReset(void);
    void StartingConfig(void);
    void StopAll(void);

    void SetWristAngle(double angle);
    double GetWristTheta(void){return wristTheta;};

    // void GatherCone(void){wristCylinderPos = frc::DoubleSolenoid::Value::kReverse;};
    // void GatherCube(void){wristCylinderPos = frc::DoubleSolenoid::Value::kForward;};
    void GatherCone(void);
    void EjectCone(void);
    void EjectConeFaster(void);

    void GatherCube(void);
    void EjectCube(void);
    void EjectCubeFaster(void); 
    void OpenJaw(void) {wristCylinder->Set(frc::DoubleSolenoid::Value::kReverse);};
    void CloseJaw(void) {wristCylinder->Set(frc::DoubleSolenoid::Value::kForward);};

    void IncreaseWristAngleBias(void) {wristAngleBias += 20.0;};
    void DecreaseWristAngleBias(void) {wristAngleBias -= 20.0;};

    void GatherReset(void){wristCylinder->Set(frc::DoubleSolenoid::Value::kOff);};

    void SetHoldState(bool holdState){holdGamepiece = holdState;};
    bool GetHoldState(void){return holdGamepiece;};

    void StopRollers(void){if (rollerCmd != CONE_INTAKE_HOLD_CMD) rollerCmd = 0.0;};
    Gamepiece GetGamepiece(){return gamepiece;};
    
    void UpdateDash(void);
    void Analyze(void);
    void Service(void);
};