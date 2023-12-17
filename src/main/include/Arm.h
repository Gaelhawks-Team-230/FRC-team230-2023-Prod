#pragma once

#include "Common.h"
#include "frc/DigitalInput.h"
#include <frc/DutyCycle.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/phoenix.h"
#include <vector>

class TalonXXV;

// control laws - arm
#define ARM_K_POS (3.0)
#define ARM_K_VEL (6.0)
#define ARM_TAU_ROBOT (0.029)
#define ARM_K_ROBOT (95.7)

#define ARM_STATOR_CURRENT_LIMIT (50.0)

// control laws - extender
#define EXTENDER_K_POS (8.0)
#define EXTENDER_K_VEL (15.0)
#define EXTENDER_TAU_ROBOT (0.018)
#define EXTENDER_K_ROBOT (64.98)

#define EXTENDER_STATOR_CURRENT_LIMIT (50.0)
#define EXTEND_DISTANCE_PER_PULSE (-5.5 / 8192.0) // 22 tooth. (inches/counts) // 22 teeth * 0.25 " per tooth = 5.5 inch / rev
#define EXTENDER_V_MAX (40.0)                     // inches per second
#define EXTENDER_V_CAL (-5.0)
// #define EXTENDER_V_CAL (-2.0)
#define ARM_MOTOR_VEL_SF ((10.0 * 360.0) / (400.0 * 2048.0))

#define INIT_ARM_ANGLE (0.0) // arm pointing straight down
#define INIT_EXTEND (0.25)

#define ARM_ENCODER_OFFSET (164.3) // was 164.3
// #define ARM_ENCODER_OFFSET (160.0) 
#define EXTENDER_ENCODER_OFFSET (0.0)

#define NUM_POSITIONS (7)

// * position defines for cone
#define CONE_ARM_STOW_ANGLE (0.0)
#define CONE_EXTENDER_STOW_POS (0.25)
#define CONE_WRIST_STOW_ANGLE (80.0)

#define CONE_ARM_PICKUP_ANGLE (26.0) //* was 28.0 before 4h fair
#define CONE_EXTENDER_PICKUP_POS (7.0) 
#define CONE_WRIST_PICKUP_ANGLE (5.0)

#define CONE_ARM_LOW_ANGLE (15.0)
#define CONE_EXTENDER_LOW_POS (0.25)
#define CONE_WRIST_LOW_ANGLE (80.0)

#define CONE_ARM_MID_ANGLE (60.0) 
#define CONE_EXTENDER_MID_POS (6.0)
#define CONE_WRIST_MID_ANGLE (60.0)
//! FIGURE OUT MID NOSE, haven't adjusted values yet
#define CONE_ARM_MID_ANGLE_NOSEMODE (85.0) 
#define CONE_EXTENDER_MID_POS_NOSEMODE (0.25)
#define CONE_WRIST_MID_ANGLE_NOSEMODE (-37.0)


#define CONE_ARM_HIGH_ANGLE (83.0)
#define CONE_EXTENDER_HIGH_POS (18.0)
#define CONE_WRIST_HIGH_ANGLE (35.0)
//! values from earlier testing, may be fine
#define CONE_ARM_HIGH_ANGLE_NOSEMODE (97.0)
#define CONE_EXTENDER_HIGH_POS_NOSEMODE (18.0)
#define CONE_WRIST_HIGH_ANGLE_NOSEMODE (-47.0)
 
#define CONE_ARM_SUB_ANGLE (103.0) // 90 for base mode
#define CONE_EXTENDER_SUB_POS (1.0)
#define CONE_WRIST_SUB_ANGLE (-42.0) // -22 for base mode

#define CONE_ARM_REARPICKUP_ANGLE (-41.3) // was -37.0 before arm offset changed to 164.3
#define CONE_EXTENDER_REARPICKUP_POS (11.0)
#define CONE_WRIST_REARPICKUP_ANGLE (-5.0)

// * position defines for cube 
#define CUBE_ARM_STOW_ANGLE (0.0)
#define CUBE_EXTENDER_STOW_POS (0.25)
#define CUBE_WRIST_STOW_ANGLE (80.0)

#define CUBE_ARM_PICKUP_ANGLE (35.0)
#define CUBE_EXTENDER_PICKUP_POS (12.0)
#define CUBE_WRIST_PICKUP_ANGLE (15.0)

#define CUBE_ARM_LOW_ANGLE (15.0)
#define CUBE_EXTENDER_LOW_POS (0.25)
#define CUBE_WRIST_LOW_ANGLE (80.0)

#define CUBE_ARM_MID_ANGLE (65.0)
#define CUBE_EXTENDER_MID_POS (1.0)

#define CUBE_ARM_HIGH_ANGLE (85.0)
#define CUBE_EXTENDER_HIGH_POS (12.0)

#define CUBE_ARM_SUB_ANGLE (103.0)
#define CUBE_EXTENDER_SUB_POS (1.0)
#define CUBE_WRIST_SUB_ANGLE (-42.0)

#define CUBE_ARM_REARPICKUP_ANGLE (-39.3) // was -35.0  before arm offset changed to 164.3
#define CUBE_EXTENDER_REARPICKUP_POS (13.0)
#define CUBE_WRIST_REARPICKUP_ANGLE (0.0)


enum MotionProfile
{
    NO_OP = 1,

    FROM_STOW_TO_HIGH = 2,
    FROM_STOW_TO_MID = 3,
    FROM_STOW_TO_LOW = 4,
    FROM_STOW_TO_PICKUP = 5,

    FROM_HIGH_TO_STOW = 6,
    FROM_HIGH_TO_MID = 7,
    FROM_HIGH_TO_LOW = 8,
    FROM_HIGH_TO_PICKUP = 9,

    FROM_MID_TO_STOW = 10,
    FROM_MID_TO_HIGH = 11,
    FROM_MID_TO_LOW = 12,
    FROM_MID_TO_PICKUP = 13,

    FROM_LOW_TO_STOW = 14,
    FROM_LOW_TO_HIGH = 15,
    FROM_LOW_TO_MID = 16,
    FROM_LOW_TO_PICKUP = 17,

    FROM_PICKUP_TO_STOW = 18,
    FROM_PICKUP_TO_HIGH = 19,
    FROM_PICKUP_TO_MID = 20,
    FROM_PICKUP_TO_LOW = 21,

    RESET_TO_STOW = 22,
    RESET_TO_PICKUP = 23,
    RESET_TO_LOW = 24,
    RESET_TO_MID = 25,
    RESET_TO_HIGH = 26,

    // substation
    FROM_STOW_TO_SUB = 27, 
    FROM_SUB_TO_STOW = 28, 
    FROM_PICKUP_TO_SUB = 29, 
    FROM_HIGH_TO_SUB = 30,

    // rear pickup
    FROM_STOW_TO_REARPICKUP = 31, 
    FROM_REARPICKUP_TO_STOW = 32, 
    FROM_REARPICKUP_TO_HIGH = 33,
    FROM_REARPICKUP_TO_MID = 34, 
    FROM_REARPICKUP_TO_LOW = 35, 

    // for autonomous
    FROM_HIGH_TO_REARPICKUP
};

class Arm
{
private:
    TalonXXV *mainRobot;

    ctre::phoenix::motorcontrol::can::WPI_TalonFX *armMotor;
    ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration armStatorCurrent;
    frc::DigitalInput *armEncoderInput;
    frc::DutyCycle *armEncoder;

    ctre::phoenix::motorcontrol::can::WPI_TalonFX *extenderMotor;
    ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration extenderStatorCurrent;
    frc::Encoder *extenderEncoder;

    // arm control vars
    double armPosCmdBias;
    double armPosCmd;
    double armPosCmdLimited;
    double armPos;
    double armPosErr;
    double armVel;
    double armVelCmd;
    double armVelErr;
    double armMotorCmd;
    double armVelErrInt;

    // extender control vars
    double extenderPosCmd;
    double extenderPosErr;
    double extenderVelCmd;
    double extenderGoalVel;
    double extenderVelErr;
    double extenderMotorCmd;
    double extenderVelErrInt;

    // extender calibration
    int calState;
    bool isExtenderCal;
    bool isCalDone;
    double calVelCmd;

    double lastArmTheta;
    double armTheta;
    double rawArmTheta;

    double lastExtenderPos;
    double extenderPos;
    double rawExtenderPos;
    double extenderVel;

    double armMotorVel;

    unsigned int serviceStage;
    unsigned int service_offset;
    unsigned int service_idx;

    std::vector<double> time_service;
    std::vector<double> arm_service;
    std::vector<double> extender_service;
    std::vector<double> wrist_service;

    bool noseMode;

    bool isMotionService;
    Position position;
    MotionProfile lookup_table[NUM_POSITIONS][NUM_POSITIONS] = {
        /*  STOW_POS                  HIGH_POS                  MID_POS                     LOW_POS             PICKUP_POS       SUBSTATION_POS     REARPICKUP_POS            */
        {RESET_TO_STOW,           FROM_STOW_TO_HIGH,       FROM_STOW_TO_MID,       FROM_STOW_TO_LOW,       FROM_STOW_TO_PICKUP, FROM_STOW_TO_SUB,    FROM_STOW_TO_REARPICKUP}, // STOW_POS
        {FROM_HIGH_TO_STOW,       NO_OP,                   FROM_HIGH_TO_MID,       FROM_HIGH_TO_LOW,       NO_OP,               NO_OP,               FROM_HIGH_TO_REARPICKUP}, // HIGH_CUBE_POS
        {FROM_MID_TO_STOW,        FROM_MID_TO_HIGH,        NO_OP,                  FROM_MID_TO_LOW,        NO_OP,               NO_OP,               NO_OP},                   // MID_CUBE_POS
        {FROM_LOW_TO_STOW,        FROM_LOW_TO_HIGH,        FROM_LOW_TO_MID,        NO_OP,                  FROM_LOW_TO_PICKUP,  NO_OP,               NO_OP},                   // LOW_CUBE_POS
        {FROM_PICKUP_TO_STOW,     FROM_PICKUP_TO_HIGH,     FROM_PICKUP_TO_MID,     FROM_PICKUP_TO_LOW,     RESET_TO_PICKUP,     FROM_PICKUP_TO_SUB,  NO_OP},                   // PICKUP_POS
        {FROM_SUB_TO_STOW,        NO_OP,                   NO_OP,                  NO_OP,                  NO_OP,               NO_OP,               NO_OP},                   // SUBSTATION_POS   
        {FROM_REARPICKUP_TO_STOW, FROM_REARPICKUP_TO_HIGH, FROM_REARPICKUP_TO_MID, FROM_REARPICKUP_TO_LOW, NO_OP,               NO_OP,               NO_OP}                    // REAR_PICKUP_POS                                                                          // SUBSTATION_POS
    };


public:
    Arm(TalonXXV *pRobot);

    void LocalReset(void);
    void StartingConfig(void);
    void StopAll(void);

    void ExtenderCalibrate(void);
    void ArmControl(void);
    void ExtenderControl(void);
    bool IsExtenderCalibrated(void){return isExtenderCal;};

    void SetArmAngle(double angle);
    void BiasArm(double bias);

    void SetExtenderPos(double extend);
    void SetExtenderVel(double goal);

    void UpdateDash(void);
    void Analyze(void);
    void Service(void);

    // * Motion profile
    void MotionService(void);
    void InversePos(void);
    void SetGoalPos(Position goal_pos);
    Position GetGoalPos(void){return position;};

    void Stow_To_Pickup(void);
    void Stow_To_Low(void);
    void Stow_To_Mid(void);
    void Stow_To_High(void);

    void Pickup_To_Low(void);
    void Pickup_To_Mid(void);
    void Pickup_To_High(void);

    void Low_To_Mid(void);
    void Low_To_High(void);
    void Mid_To_High(void);

    void Reset_To_Stow(void);
    void Reset_To_Pickup(void);
    void Reset_To_Low(void);
    void Reset_To_Mid(void);
    void Reset_To_High(void);

    void Stow_To_Sub(void); 
    void Pickup_To_Sub(void); 
    void High_To_Sub(void);

    void Stow_To_RearPickup(void); 
    void RearPickup_To_Stow(void);
    void RearPickup_To_High(void);
    void RearPickup_To_Mid(void);
    void RearPickup_To_Low(void);

    void High_To_RearPickup(void);

    void Stow_To_Mid_Nose_Mode(void);
    void Stow_To_High_Nose_Mode(void);
    //! I do not think we will need profiles connecting nose mode high/mid to our pickup/rearpickup.
    //! Perhaps only when going from mid/high to pickup in case we drop a gamepiece but even then I don't know how necessary this is.
    // void Pickup_To_Mid_Nose_Mode(void);
    // void Pickup_To_High_Nose_Mode(void);
    void Low_To_Mid_Nose_Mode(void);
    void Low_To_High_Nose_Mode(void);
    void Mid_To_High_Nose_Mode(void);


    void SetNoseMode(void);
    void ClearNoseMode(void);
    bool IsNoseMode();
};