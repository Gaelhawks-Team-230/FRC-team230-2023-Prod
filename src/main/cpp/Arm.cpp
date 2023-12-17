#include "Common.h"
#include "TalonXXV.h"
#include "Arm.h"
#include "Joystick.h"
#include <algorithm>

Arm::Arm(TalonXXV *pRobot)
{
    mainRobot = pRobot;

    armMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(CAN_ARM_MOTOR);
    extenderMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(CAN_EXTENDER_MOTOR);

    // absolute encoder (arm)
    armEncoderInput = new frc::DigitalInput(ARM_ENCODER);
    armEncoder = new frc::DutyCycle(armEncoderInput);

    // incremental encoder (extender)
    extenderEncoder = new frc::Encoder(EXTENDER_ENCODER_A, EXTENDER_ENCODER_B, false);
    extenderEncoder->SetDistancePerPulse(1.0);

    StartingConfig();
}

void Arm::LocalReset()
{
    // * moved to startingconfig
    // armPosCmd = INIT_ARM_ANGLE;
    // armPos = 0.0;
    armPosCmdBias = 0.0;
    armVel = 0.0;
    armVelCmd = 0.0;
    armVelErr = 0.0;
    armMotorCmd = 0.0;
    armPosErr = 0.0;
    armVelErrInt = 0.0;

    extenderPos = 0.0;
    extenderVel = 0.0;
    extenderVelCmd = 0.0;
    extenderVelErr = 0.0;
    extenderMotorCmd = 0.0;
    extenderPosErr = 0.0;
    extenderVelErrInt = 0.0;

    calState = 0;
    calVelCmd = 0.0;

    extenderPos = extenderEncoder->GetRaw();
    rawExtenderPos = EXTEND_DISTANCE_PER_PULSE * (extenderEncoder->GetRaw()); // - sign for backwards encoder
    extenderPos = rawExtenderPos - EXTENDER_ENCODER_OFFSET;
    lastExtenderPos = extenderPos;

    extenderVel = 0.0;
    serviceStage = 0;
    service_offset = 0;
    service_idx = 0;
    isMotionService = false;

    time_service.clear();
    arm_service.clear();
    extender_service.clear();
    wrist_service.clear();

    noseMode = false;
    // position = STOW_POS;
}

void Arm::StartingConfig()
{
#ifdef CTRE_FACTORY_RESET
    armMotor->ConfigFactoryDefault();
    extenderMotor->ConfigFactoryDefault();
#endif

    armMotor->SetNeutralMode(NeutralMode::Brake);
    armPosCmd = INIT_ARM_ANGLE;
    armPos = 0.0;
    armPosCmdBias = 0.0;

    position = STOW_POS;

    // extenderMotor->ConfigFactoryDefault();
    isExtenderCal = false;
    isCalDone = false;
    extenderMotor->SetNeutralMode(NeutralMode::Brake);

    armStatorCurrent.currentLimit = ARM_STATOR_CURRENT_LIMIT;
    armStatorCurrent.enable = true;
    extenderMotor->ConfigStatorCurrentLimit(armStatorCurrent);

    extenderStatorCurrent.currentLimit = EXTENDER_STATOR_CURRENT_LIMIT;
    extenderStatorCurrent.enable = true;
    extenderMotor->ConfigStatorCurrentLimit(extenderStatorCurrent);

    extenderPosCmd = INIT_EXTEND;

    LocalReset();
}

void Arm::StopAll()
{
    LocalReset();
}

void Arm::ExtenderCalibrate()
{
    switch (calState)
    {
    case 0:
        calVelCmd = EXTENDER_V_CAL;
        extenderPosCmd = INIT_EXTEND;
        calState++;
        break;

    case 1:
        // the conditions that cause stalling - determine later when testing.
        if ((extenderMotor->GetStatorCurrent() > 20.0) && (fabs(extenderVel) < 0.5 * fabs(EXTENDER_V_CAL)))
        {
            // printf("Calibrated\n");
            calState++;
        }
        break;

    case 2:
        calVelCmd = 0.0;
        extenderEncoder->Reset();
        isExtenderCal = true;
        // if we ever have to recal this will need to be set back to false
        break;
    }
}

void Arm::SetArmAngle(double angle)
{
    armPosCmd = angle;
}

void Arm::BiasArm(double bias)
{
    armPosCmdBias = bias;
}

void Arm::SetExtenderPos(double extend)
{
    extenderPosCmd = extend;
}

void Arm::SetExtenderVel(double goal)
{
    extenderGoalVel = goal;
}

void Arm::SetNoseMode()
{
    noseMode = true;
}
void Arm::ClearNoseMode()
{
    noseMode = false;
}
bool Arm::IsNoseMode()
{
    return noseMode;
}

// must read encoders and store vals here
// read pos and vel
void Arm::Analyze()
{
    // calculations for arm
    lastArmTheta = armTheta;
    rawArmTheta = 360.0 * (armEncoder->GetOutput() - DMIN) / (DMAX - DMIN);
    // armTheta = rawArmTheta - ARM_ENCODER_OFFSET;
    armTheta = -(rawArmTheta - ARM_ENCODER_OFFSET); // encoder reads backwards
    if (armTheta > 180.0)
    {
        armTheta -= 360.0;
    }
    else if (armTheta < -180.0)
    {
        armTheta += 360;
    }

    armVel = (armTheta - lastArmTheta) / LOOPTIME;
    armVel = TalonXXV::Limit(-200.0, 200.0, armVel);
    armMotorVel = (armMotor->GetSelectedSensorVelocity()) * ARM_MOTOR_VEL_SF;

    // calculations for extender
    lastExtenderPos = extenderPos;
    rawExtenderPos = EXTEND_DISTANCE_PER_PULSE * (extenderEncoder->GetRaw()); // - sign for backwards encoder
    extenderPos = rawExtenderPos - EXTENDER_ENCODER_OFFSET;

    extenderVel = (extenderPos - lastExtenderPos) / LOOPTIME;
    extenderVel = TalonXXV::Limit(-100.0, 100.0, extenderVel);
}

// control system for arm
void Arm::ArmControl()
{
    // armPosCmd = 70.0*(mainRobot->m_userInput->GetGamepad_RAW_Y());
    // armPosCmd = 0.0;
    armPosCmdLimited = TalonXXV::Limit(-60.0, 103.0, (armPosCmd + armPosCmdBias));
    armPosErr = armPosCmdLimited - armTheta;
    armVelCmd = armPosErr * ARM_K_POS;
    // armVelCmd = 30.0*(mainRobot->m_userInput->GetFlightCtrl_RAW_Z());
    // armVelErr = armVelCmd - armVel;
    armVelErr = armVelCmd - armMotorVel;
    armVelErrInt = armVelErrInt + armVelErr * LOOPTIME;
    armVelErrInt = TalonXXV::Limit(-ARM_K_ROBOT / ARM_K_VEL, ARM_K_ROBOT / ARM_K_VEL, armVelErrInt);
    armMotorCmd = ARM_K_VEL / ARM_K_ROBOT * (ARM_TAU_ROBOT * armVelErr + armVelErrInt);
    armMotorCmd = TalonXXV::Limit(-1.0, 1.0, armMotorCmd);
    // armMotorCmd = 0.2*(mainRobot->m_userInput->GetFlightCtrl_RAW_Z());
    armMotor->Set(armMotorCmd);
    static double maxCurrent = 0.0;
    static double localCurrent = 0.0;
    static double maxMotorCmd = 0.0;
    localCurrent = armMotor->GetStatorCurrent();
    maxCurrent = fmax(fabs(localCurrent), maxCurrent);
    maxMotorCmd = fmax(fabs(armMotorCmd), maxMotorCmd);
    // printf("Arm Current: %f %f %f\n", localCurrent, maxCurrent, maxMotorCmd);

}

// control system for extender
void Arm::ExtenderControl()
{
    // extenderPosCmd = 5.0 + 2.0*(mainRobot->m_userInput->GetGamepad_RAW_R());
    extenderPosErr = extenderPosCmd - extenderPos;
    extenderVelCmd = extenderPosErr * EXTENDER_K_POS;
    extenderVelCmd = TalonXXV::Limit(-EXTENDER_V_MAX, EXTENDER_V_MAX, extenderVelCmd);
    if (calVelCmd)
    {
        extenderVelCmd = calVelCmd;
    }

    // extenderVelCmd = 5.0*(mainRobot->m_userInput->GetFlightCtrl_RAW_Z());

    extenderVelErr = extenderVelCmd - extenderVel;
    extenderVelErrInt = extenderVelErrInt + extenderVelErr * LOOPTIME;
    extenderVelErrInt = TalonXXV::Limit(0.75 * (-EXTENDER_K_ROBOT / EXTENDER_K_VEL), 0.75 * (EXTENDER_K_ROBOT / EXTENDER_K_VEL), extenderVelErrInt);
    extenderMotorCmd = EXTENDER_K_VEL / EXTENDER_K_ROBOT * (EXTENDER_TAU_ROBOT * extenderVelErr + extenderVelErrInt);
    extenderMotorCmd = TalonXXV::Limit(-0.75, 0.75, extenderMotorCmd);
    extenderMotor->Set(extenderMotorCmd);
}

void Arm::SetGoalPos(Position goal_position)
{
    if (isMotionService)
    {
        return;
    }

    MotionProfile profile = lookup_table[position][goal_position];
    switch (profile)
    {
    case NO_OP:
        break;

    case FROM_STOW_TO_HIGH: //* DUAL CONE
        if (noseMode)
            Stow_To_High_Nose_Mode();
        else
            Stow_To_High();
        position = HIGH_POS;
        isMotionService = true;
        break;
    case FROM_STOW_TO_MID: //* DUAL CONE
        if (noseMode)
            Stow_To_Mid_Nose_Mode();
        else
            Stow_To_Mid();
        position = MID_POS;
        isMotionService = true;
        break;
    case FROM_STOW_TO_LOW:
        Stow_To_Low();
        position = LOW_POS;
        isMotionService = true;
        break;
    case FROM_STOW_TO_PICKUP:
        Stow_To_Pickup();
        position = PICKUP_POS;
        isMotionService = true;
        break;

    case FROM_HIGH_TO_STOW: //* DUAL CONE
        if (noseMode)
            Stow_To_High_Nose_Mode();
        else
            Stow_To_High();
        InversePos();
        position = STOW_POS;
        isMotionService = true;
        break;
    case FROM_HIGH_TO_MID: //* DUAL CONE
        if (noseMode)
            Mid_To_High_Nose_Mode();
        else
            Mid_To_High();
        InversePos();
        position = MID_POS;
        isMotionService = true;
        break;
    case FROM_HIGH_TO_LOW: //* DUAL CONE
        if (noseMode)
            Low_To_High_Nose_Mode();
        else
            Low_To_High();
        InversePos();
        position = LOW_POS;
        isMotionService = true;
        break;
    case FROM_HIGH_TO_PICKUP: 
        Pickup_To_High();
        InversePos();
        position = PICKUP_POS;
        isMotionService = true;
        break;

    case FROM_MID_TO_STOW: //* DUAL CONE
        if (noseMode)
            Stow_To_Mid_Nose_Mode();
        else
            Stow_To_Mid();
        InversePos();
        position = STOW_POS;
        isMotionService = true;
        break;
    case FROM_MID_TO_HIGH: //* DUAL CONE
        if (noseMode)
            Mid_To_High_Nose_Mode();
        else
            Mid_To_High();
        position = HIGH_POS;
        isMotionService = true;
        break;
    case FROM_MID_TO_LOW: //* DUAL CONE
        if (noseMode)
            Low_To_Mid_Nose_Mode();
        else
            Low_To_Mid();
        InversePos();
        position = LOW_POS;
        isMotionService = true;
        break;
    case FROM_MID_TO_PICKUP:
        Pickup_To_Mid();
        InversePos();
        position = PICKUP_POS;
        isMotionService = true;
        break;

    case FROM_LOW_TO_STOW:
        Stow_To_Low();
        InversePos();
        position = STOW_POS;
        isMotionService = true;
        break;
    case FROM_LOW_TO_HIGH: //* DUAL CONE
        if (noseMode)
            Low_To_High_Nose_Mode();
        else
            Low_To_High();
        position = HIGH_POS;
        isMotionService = true;
        break;
    case FROM_LOW_TO_MID: //* DUAL CONE
        if (noseMode)
            Low_To_Mid_Nose_Mode();
        else
            Low_To_Mid();
        position = MID_POS;
        isMotionService = true;
        break;
    case FROM_LOW_TO_PICKUP: 
        Pickup_To_Low();
        InversePos();
        position = PICKUP_POS;
        isMotionService = true;
        break;

    case FROM_PICKUP_TO_STOW:
        Stow_To_Pickup();
        InversePos();
        position = STOW_POS;
        isMotionService = true;
        break;
    case FROM_PICKUP_TO_HIGH:
        Pickup_To_High();
        position = HIGH_POS;
        isMotionService = true;
        break;
    case FROM_PICKUP_TO_MID: 
        Pickup_To_Mid();
        position = MID_POS;
        isMotionService = true;
        break;
    case FROM_PICKUP_TO_LOW:
        Pickup_To_Low();
        position = LOW_POS;
        isMotionService = true;
        break;

    case RESET_TO_STOW:
        Reset_To_Stow();
        position = STOW_POS;
        break;
    case RESET_TO_PICKUP:
        Reset_To_Pickup();
        position = PICKUP_POS;
        break;
    case RESET_TO_HIGH:
        Reset_To_High();
        position = HIGH_POS;
        break;
    case RESET_TO_MID:
        Reset_To_Mid();
        position = MID_POS;
        break;
    case RESET_TO_LOW:
        Reset_To_Low();
        position = LOW_POS;
        break;

    case FROM_STOW_TO_SUB:
        Stow_To_Sub();
        position = SUB_POS;
        isMotionService = true;
        break; 
    case FROM_SUB_TO_STOW: 
        Stow_To_Sub();
        InversePos(); 
        position = STOW_POS; 
        isMotionService = true;
        break; 
    case FROM_PICKUP_TO_SUB: 
        Pickup_To_Sub();
        position = SUB_POS; 
        isMotionService = true;
        break; 
    case FROM_HIGH_TO_SUB:
        High_To_Sub();
        position = SUB_POS;
        isMotionService = true;
        break; 

    case FROM_STOW_TO_REARPICKUP: 
        Stow_To_RearPickup(); 
        position = REARPICKUP_POS; 
        isMotionService = true; 
        break; 
    case FROM_REARPICKUP_TO_STOW:
        RearPickup_To_Stow(); 
        position = STOW_POS; 
        isMotionService = true; 
        break; 
    case FROM_REARPICKUP_TO_HIGH: 
        RearPickup_To_High();
        position = HIGH_POS; 
        isMotionService = true; 
        break;
    case FROM_REARPICKUP_TO_MID: 
        RearPickup_To_Mid();
        position = MID_POS; 
        isMotionService = true; 
        break;
    case FROM_REARPICKUP_TO_LOW: 
        RearPickup_To_Low();
        position = LOW_POS; 
        isMotionService = true; 
        break;
    case FROM_HIGH_TO_REARPICKUP:
        High_To_RearPickup(); 
        position = REARPICKUP_POS;
        isMotionService = true;
        break;
    }
    
        
}
void Arm::Stow_To_Pickup()
{
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleEnd = CONE_ARM_PICKUP_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_PICKUP_POS;
        l_wristAngleEnd = CONE_WRIST_PICKUP_ANGLE;
        break;
    case CUBE:
        l_armAngleEnd = CUBE_ARM_PICKUP_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_PICKUP_POS;
        l_wristAngleEnd = CUBE_WRIST_PICKUP_ANGLE;
        break;
    default:
        l_armAngleEnd = CONE_ARM_PICKUP_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_PICKUP_POS;
        l_wristAngleEnd = CONE_WRIST_PICKUP_ANGLE;
        break;
    }
    time_service = {0.0, 0.5, 0.9};
    arm_service = {CONE_ARM_STOW_ANGLE, 45.0, l_armAngleEnd};
    extender_service = {CONE_EXTENDER_STOW_POS, 0.5, l_extenderPosEnd};
    wrist_service = {CONE_WRIST_STOW_ANGLE, 40.0, l_wristAngleEnd};
}
void Arm::Stow_To_Low()
{
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleEnd = CONE_ARM_LOW_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_LOW_POS;
        l_wristAngleEnd = CONE_WRIST_LOW_ANGLE;
        break;
    case CUBE:
        l_armAngleEnd = CUBE_ARM_LOW_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_LOW_POS;
        l_wristAngleEnd = CUBE_WRIST_LOW_ANGLE;
        break;
    default:
        l_armAngleEnd = CONE_ARM_LOW_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_LOW_POS;
        l_wristAngleEnd = CONE_WRIST_LOW_ANGLE;
        break;
    }
    time_service = {0.0, 0.3};
    arm_service = {CONE_ARM_STOW_ANGLE, l_armAngleEnd};
    extender_service = {CONE_EXTENDER_STOW_POS, l_extenderPosEnd};
    wrist_service = {CONE_WRIST_STOW_ANGLE, l_wristAngleEnd};
}
void Arm::Stow_To_Mid()
{
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleEnd = CONE_ARM_MID_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_MID_POS;
        l_wristAngleEnd = CONE_WRIST_MID_ANGLE;
        break;
    case CUBE:
        l_armAngleEnd = CUBE_ARM_MID_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_MID_POS;
        l_wristAngleEnd = 0.0;
        break;
    default:
        l_armAngleEnd = CONE_ARM_MID_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_MID_POS;
        l_wristAngleEnd = CONE_WRIST_MID_ANGLE;
        break;
    }
    time_service = {0.0, 0.5, 1.0};
    arm_service = {CONE_ARM_STOW_ANGLE, 65.0, l_armAngleEnd};
    extender_service = {CONE_EXTENDER_STOW_POS, 1.0, l_extenderPosEnd};
    wrist_service = {CONE_WRIST_STOW_ANGLE, 80.0, l_wristAngleEnd};
}
void Arm::Stow_To_High()
{
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleEnd = CONE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = CONE_WRIST_HIGH_ANGLE;
        break;
    case CUBE:
        l_armAngleEnd = CUBE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = 0.0;
        break;
    default:
        l_armAngleEnd = CONE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = CONE_WRIST_HIGH_ANGLE;
        break;
    }
    time_service = {0.0, 0.75, 1.0};
    arm_service = {CONE_ARM_STOW_ANGLE, 83.0, l_armAngleEnd};
    extender_service = {CONE_EXTENDER_STOW_POS, 1.0, l_extenderPosEnd};
    wrist_service = {CONE_WRIST_STOW_ANGLE, 80.0, l_wristAngleEnd};
}

void Arm::Pickup_To_Low()
{
    double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleStart = CONE_ARM_PICKUP_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_PICKUP_POS;
        l_wristAngleStart = CONE_WRIST_PICKUP_ANGLE;
        l_armAngleEnd = CONE_ARM_LOW_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_LOW_POS;
        l_wristAngleEnd = CONE_WRIST_LOW_ANGLE;
        break;
    case CUBE:
        l_armAngleStart = CUBE_ARM_PICKUP_ANGLE;
        l_extenderPosStart = CUBE_EXTENDER_PICKUP_POS;
        l_wristAngleStart = CUBE_WRIST_PICKUP_ANGLE;
        l_armAngleEnd = CUBE_ARM_LOW_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_LOW_POS;
        l_wristAngleEnd = CUBE_WRIST_LOW_ANGLE;
        break;
    default:
        l_armAngleStart = CONE_ARM_PICKUP_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_PICKUP_POS;
        l_wristAngleStart = CONE_WRIST_PICKUP_ANGLE;
        l_armAngleEnd = CONE_ARM_LOW_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_LOW_POS;
        l_wristAngleEnd = CONE_WRIST_LOW_ANGLE;
        break;
    }
    time_service = {0.0, 0.5, 0.9};
    arm_service = {l_armAngleStart, 45.0, l_armAngleEnd};
    extender_service = {l_extenderPosStart, 0.5, l_extenderPosEnd};
    wrist_service = {l_wristAngleStart, 40.0, l_wristAngleEnd};
}
void Arm::Pickup_To_Mid()
{
    double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleStart = CONE_ARM_PICKUP_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_PICKUP_POS;
        l_wristAngleStart = CONE_WRIST_PICKUP_ANGLE;
        l_armAngleEnd = CONE_ARM_MID_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_MID_POS;
        l_wristAngleEnd = CONE_WRIST_MID_ANGLE;
        break;
    case CUBE:
        l_armAngleStart = CUBE_ARM_PICKUP_ANGLE;
        l_extenderPosStart = CUBE_EXTENDER_PICKUP_POS;
        l_wristAngleStart = CUBE_WRIST_PICKUP_ANGLE;
        l_armAngleEnd = CUBE_ARM_MID_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_MID_POS;
        l_wristAngleEnd = 0.0;
        break;
    default:
        l_armAngleStart = CONE_ARM_PICKUP_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_PICKUP_POS;
        l_wristAngleStart = CONE_WRIST_PICKUP_ANGLE;
        l_armAngleEnd = CONE_ARM_MID_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_MID_POS;
        l_wristAngleEnd = CONE_WRIST_MID_ANGLE;
        break;
    }
    time_service = {0.0, 0.5};
    arm_service = {l_armAngleStart, l_armAngleEnd};
    extender_service = {l_extenderPosStart, l_extenderPosEnd};
    wrist_service = {l_wristAngleStart, l_wristAngleEnd};
}
void Arm::Pickup_To_High()
{
    double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleStart = CONE_ARM_PICKUP_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_PICKUP_POS;
        l_wristAngleStart = CONE_WRIST_PICKUP_ANGLE;
        l_armAngleEnd = CONE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = CONE_WRIST_HIGH_ANGLE;
        break;
    case CUBE:
        l_armAngleStart = CUBE_ARM_PICKUP_ANGLE;
        l_extenderPosStart = CUBE_EXTENDER_PICKUP_POS;
        l_wristAngleStart = CUBE_WRIST_PICKUP_ANGLE;
        l_armAngleEnd = CUBE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = 0.0;
        break;
    default:
        l_armAngleStart = CONE_ARM_PICKUP_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_PICKUP_POS;
        l_wristAngleStart = CONE_WRIST_PICKUP_ANGLE;
        l_armAngleEnd = CONE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = CONE_WRIST_HIGH_ANGLE;
        break;
    }
    time_service = {0.0, 0.5};
    arm_service = {l_armAngleStart, l_armAngleEnd};
    extender_service = {l_extenderPosStart, l_extenderPosEnd};
    wrist_service = {l_wristAngleStart, l_wristAngleEnd};
}

void Arm::Low_To_Mid()
{
    double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleStart = CONE_ARM_LOW_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_LOW_POS;
        l_wristAngleStart = CONE_WRIST_LOW_ANGLE;
        l_armAngleEnd = CONE_ARM_MID_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_MID_POS;
        l_wristAngleEnd = CONE_WRIST_MID_ANGLE;
        break;
    case CUBE:
        l_armAngleStart = CUBE_ARM_LOW_ANGLE;
        l_extenderPosStart = CUBE_EXTENDER_LOW_POS;
        l_wristAngleStart = CUBE_WRIST_LOW_ANGLE;
        l_armAngleEnd = CUBE_ARM_MID_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_MID_POS;
        l_wristAngleEnd = 0.0;
        break;
    default:
        l_armAngleStart = CONE_ARM_LOW_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_LOW_POS;
        l_wristAngleStart = CONE_WRIST_LOW_ANGLE;
        l_armAngleEnd = CONE_ARM_MID_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_MID_POS;
        l_wristAngleEnd = CONE_WRIST_MID_ANGLE;
        break;
    }
    time_service = {0.0, 0.5};
    arm_service = {l_armAngleStart, l_armAngleEnd};
    extender_service = {l_extenderPosStart, l_extenderPosEnd};
    wrist_service = {l_wristAngleStart, l_wristAngleEnd};
}
void Arm::Low_To_High()
{
    double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleStart = CONE_ARM_LOW_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_LOW_POS;
        l_wristAngleStart = CONE_WRIST_LOW_ANGLE;
        l_armAngleEnd = CONE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = CONE_WRIST_HIGH_ANGLE;
        break;
    case CUBE:
        l_armAngleStart = CUBE_ARM_LOW_ANGLE;
        l_extenderPosStart = CUBE_EXTENDER_LOW_POS;
        l_wristAngleStart = CUBE_WRIST_LOW_ANGLE;
        l_armAngleEnd = CUBE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = 0.0;
        break;
    default:
        l_armAngleStart = CONE_ARM_LOW_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_LOW_POS;
        l_wristAngleStart = CONE_WRIST_LOW_ANGLE;
        l_armAngleEnd = CONE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = CONE_WRIST_HIGH_ANGLE;
        break;
    }
    time_service = {0.0, 0.5};
    arm_service = {l_armAngleStart, l_armAngleEnd};
    extender_service = {l_extenderPosStart, l_extenderPosEnd};
    wrist_service = {l_wristAngleStart, l_wristAngleEnd};
}
void Arm::Mid_To_High()
{
    double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleStart = CONE_ARM_MID_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_MID_POS;
        l_wristAngleStart = CONE_WRIST_MID_ANGLE;
        l_armAngleEnd = CONE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = CONE_WRIST_HIGH_ANGLE;
        break;
    case CUBE:
        l_armAngleStart = CUBE_ARM_MID_ANGLE;
        l_extenderPosStart = CUBE_EXTENDER_MID_POS;
        l_wristAngleStart = 0.0;
        l_armAngleEnd = CUBE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = 0.0;
        break;
    default:
        l_armAngleStart = CONE_ARM_MID_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_MID_POS;
        l_wristAngleStart = CONE_WRIST_MID_ANGLE;
        l_armAngleEnd = CONE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = CONE_WRIST_HIGH_ANGLE;
        break;
    }
    time_service = {0.0, 0.5};
    arm_service = {l_armAngleStart, l_armAngleEnd};
    extender_service = {l_extenderPosStart, l_extenderPosEnd};
    wrist_service = {l_wristAngleStart, l_wristAngleEnd};
}

void Arm::Reset_To_Stow()
{
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        armPosCmd = CONE_ARM_STOW_ANGLE;
        extenderPosCmd = CONE_EXTENDER_STOW_POS;
        mainRobot->m_grabber->SetWristAngle(CONE_WRIST_STOW_ANGLE);
        break;
    case CUBE:
        armPosCmd = CUBE_ARM_STOW_ANGLE;
        extenderPosCmd = CUBE_EXTENDER_STOW_POS;
        mainRobot->m_grabber->SetWristAngle(CUBE_WRIST_STOW_ANGLE);
        break;
    default:
        armPosCmd = CONE_ARM_STOW_ANGLE;
        extenderPosCmd = CONE_EXTENDER_STOW_POS;
        mainRobot->m_grabber->SetWristAngle(CONE_WRIST_STOW_ANGLE);
        break;
    }
}
void Arm::Reset_To_Pickup()
{
    // double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    // double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();

    // l_armAngleStart = armTheta;
    // l_extenderPosStart = extenderPos;
    // l_wristAngleStart = mainRobot->m_grabber->GetWristTheta();

    switch (l_gamepiece)
    {
    case CONE:
        armPosCmd = CONE_ARM_PICKUP_ANGLE;
        extenderPosCmd = CONE_EXTENDER_PICKUP_POS;
        mainRobot->m_grabber->SetWristAngle(CONE_WRIST_PICKUP_ANGLE);
        // l_armAngleEnd = CONE_ARM_PICKUP_ANGLE;
        // l_extenderPosEnd = CONE_EXTENDER_PICKUP_POS;
        // l_wristAngleEnd = CONE_WRIST_PICKUP_ANGLE;
        break;
    case CUBE:
        armPosCmd = CUBE_ARM_PICKUP_ANGLE;
        extenderPosCmd = CUBE_EXTENDER_PICKUP_POS;
        mainRobot->m_grabber->SetWristAngle(CUBE_WRIST_PICKUP_ANGLE);
        // l_armAngleEnd = CUBE_ARM_PICKUP_ANGLE;
        // l_extenderPosEnd = CUBE_EXTENDER_PICKUP_POS;
        // l_wristAngleEnd = CUBE_WRIST_PICKUP_ANGLE;
        break;
    default:
        armPosCmd = CONE_ARM_PICKUP_ANGLE;
        extenderPosCmd = CONE_EXTENDER_PICKUP_POS;
        mainRobot->m_grabber->SetWristAngle(CONE_WRIST_PICKUP_ANGLE);
        // l_armAngleEnd = CONE_ARM_PICKUP_ANGLE;
        // l_extenderPosEnd = CONE_EXTENDER_PICKUP_POS;
        // l_wristAngleEnd = CONE_WRIST_PICKUP_ANGLE;
        break;
    }
    // time_service = {0.0, 0.3, 0.5};
    // arm_service = {l_armAngleEnd, 30.0, l_armAngleEnd};
    // extender_service = {l_extenderPosEnd, 7.0, l_extenderPosEnd};
    // wrist_service = {l_wristAngleEnd, 15.0, l_wristAngleEnd};
}
void Arm::Reset_To_Low()
{
}
void Arm::Reset_To_Mid()
{
}
void Arm::Reset_To_High()
{
}

void Arm::Stow_To_Sub()
{
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleEnd = CONE_ARM_SUB_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_SUB_POS;
        l_wristAngleEnd = CONE_WRIST_SUB_ANGLE;
        break;
    case CUBE:
        l_armAngleEnd = CUBE_ARM_SUB_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_SUB_POS;
        l_wristAngleEnd = CUBE_WRIST_SUB_ANGLE;
        break;
    default:
        l_armAngleEnd = CONE_ARM_SUB_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_SUB_POS;
        l_wristAngleEnd = CONE_WRIST_SUB_ANGLE;
        break;
    }
    time_service = {0.0, 0.75, 1.0};
    arm_service = {CONE_ARM_STOW_ANGLE, 70.0, l_armAngleEnd};
    extender_service = {CONE_EXTENDER_STOW_POS, 1.0, l_extenderPosEnd};
    wrist_service = {CONE_WRIST_STOW_ANGLE, 0.0, l_wristAngleEnd};
}
void Arm::Pickup_To_Sub()
{
    double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleStart = CONE_ARM_PICKUP_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_PICKUP_POS;
        l_wristAngleStart = CONE_WRIST_PICKUP_ANGLE;
        l_armAngleEnd = CONE_ARM_SUB_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_SUB_POS;
        l_wristAngleEnd = CONE_WRIST_SUB_ANGLE;
        break;
    case CUBE:
        l_armAngleStart = CUBE_ARM_PICKUP_ANGLE;
        l_extenderPosStart = CUBE_EXTENDER_PICKUP_POS;
        l_wristAngleStart = CUBE_WRIST_PICKUP_ANGLE;
        l_armAngleEnd = CUBE_ARM_SUB_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_SUB_POS;
        l_wristAngleEnd = CUBE_WRIST_SUB_ANGLE;
        break;
    default:
        l_armAngleStart = CONE_ARM_PICKUP_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_PICKUP_POS;
        l_wristAngleStart = CONE_WRIST_PICKUP_ANGLE;
        l_armAngleEnd = CONE_ARM_SUB_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_SUB_POS;
        l_wristAngleEnd = CONE_WRIST_SUB_ANGLE;
        break;
    }
    time_service = {0.0, 0.5};
    arm_service = {l_armAngleStart, l_armAngleEnd};
    extender_service = {l_extenderPosStart, l_extenderPosEnd};
    wrist_service = {l_wristAngleStart, l_wristAngleEnd};
}
void Arm::High_To_Sub()
{
    double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleStart = CONE_ARM_HIGH_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_HIGH_POS;
        l_wristAngleStart = CONE_WRIST_HIGH_ANGLE;
        l_armAngleEnd = CONE_ARM_SUB_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_SUB_POS;
        l_wristAngleEnd = CONE_WRIST_SUB_ANGLE;
        break;
    case CUBE:
        l_armAngleStart = CUBE_ARM_HIGH_ANGLE;
        l_extenderPosStart = CUBE_EXTENDER_HIGH_POS;
        l_wristAngleStart = 0.0;
        l_armAngleEnd = CUBE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = 0.0;
        break;
    default:
        l_armAngleStart = CONE_ARM_HIGH_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_HIGH_POS;
        l_wristAngleStart = CONE_WRIST_HIGH_ANGLE;
        l_armAngleEnd = CONE_ARM_SUB_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_SUB_POS;
        l_wristAngleEnd = CONE_WRIST_SUB_ANGLE;
        break;
    }
    time_service = {0.0, 0.5};
    arm_service = {l_armAngleStart, l_armAngleEnd};
    extender_service = {l_extenderPosStart, l_extenderPosEnd};
    wrist_service = {l_wristAngleStart, l_wristAngleEnd};
}

void Arm::Stow_To_RearPickup()
{
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    { 
    case CONE:
        l_armAngleEnd = CONE_ARM_REARPICKUP_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_REARPICKUP_POS;
        l_wristAngleEnd = CONE_WRIST_REARPICKUP_ANGLE;
        break;
    case CUBE:
        l_armAngleEnd = CUBE_ARM_REARPICKUP_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_REARPICKUP_POS;
        l_wristAngleEnd = CUBE_WRIST_REARPICKUP_ANGLE;
        break;
    default:
        l_armAngleEnd = CONE_ARM_REARPICKUP_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_REARPICKUP_POS;
        l_wristAngleEnd = CONE_WRIST_REARPICKUP_ANGLE;
        break;
    }
    time_service = {0.0, 0.8, 1.0}; 
    arm_service = {CONE_ARM_STOW_ANGLE, -34.3, l_armAngleEnd}; // intermediate was -30.0 before arm offset changed to 164.3
    extender_service = {CONE_EXTENDER_STOW_POS, 0.5, l_extenderPosEnd};
    wrist_service = {CONE_WRIST_STOW_ANGLE, -40.0, l_wristAngleEnd};
}
void Arm::RearPickup_To_Stow()
{
    double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    { 
    case CONE:
        l_armAngleStart = CONE_ARM_REARPICKUP_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_REARPICKUP_POS;
        l_wristAngleStart = CONE_WRIST_REARPICKUP_ANGLE;
        l_armAngleEnd = CONE_ARM_STOW_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_STOW_POS;
        l_wristAngleEnd = CONE_WRIST_STOW_ANGLE;
        break;
    case CUBE:
        l_armAngleStart = CUBE_ARM_REARPICKUP_ANGLE;
        l_extenderPosStart = CUBE_EXTENDER_REARPICKUP_POS;
        l_wristAngleStart = CUBE_WRIST_REARPICKUP_ANGLE;
        l_armAngleEnd = CUBE_ARM_STOW_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_STOW_POS;
        l_wristAngleEnd = CUBE_WRIST_STOW_ANGLE;
        break;
    default:
        l_armAngleStart = CONE_ARM_REARPICKUP_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_REARPICKUP_POS;
        l_wristAngleStart = CONE_WRIST_REARPICKUP_ANGLE;
        l_armAngleEnd = CONE_ARM_STOW_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_STOW_POS;
        l_wristAngleEnd = CONE_WRIST_STOW_ANGLE;
        break;
    }
    time_service = {0.0, 0.4, 0.8, 1.5};
    arm_service = {l_armAngleStart, -65.0, -60.0, l_armAngleEnd};
    extender_service = {l_extenderPosStart, 0.25, 0.25, l_extenderPosEnd};
    wrist_service = {l_wristAngleStart, 60.0, 85.0, l_wristAngleEnd};
}
void Arm::RearPickup_To_High()
{
    double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    { 
    case CONE:
        l_armAngleStart = CONE_ARM_REARPICKUP_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_REARPICKUP_POS;
        l_wristAngleStart = CONE_WRIST_REARPICKUP_ANGLE;
        l_armAngleEnd = CONE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = CONE_WRIST_HIGH_ANGLE;
        break;
    case CUBE:
        l_armAngleStart = CUBE_ARM_REARPICKUP_ANGLE;
        l_extenderPosStart = CUBE_EXTENDER_REARPICKUP_POS;
        l_wristAngleStart = CUBE_WRIST_REARPICKUP_ANGLE;
        l_armAngleEnd = CUBE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = 0.0;
        break;
    default:
        l_armAngleStart = CONE_ARM_REARPICKUP_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_REARPICKUP_POS;
        l_wristAngleStart = CONE_WRIST_REARPICKUP_ANGLE;
        l_armAngleEnd = CONE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = CONE_WRIST_HIGH_ANGLE;
        break;
    }
    time_service = {0.0, 0.5, 0.8, 1.5, 1.6};
    arm_service = {l_armAngleStart, -60.0, 0.0, 70.0, l_armAngleEnd};
    extender_service = {l_extenderPosStart, 0.25, 0.25, 0.25,  l_extenderPosEnd};
    wrist_service = {l_wristAngleStart, 95.0, 95.0, 95.0, l_wristAngleEnd}; 
    // time_service = {0.0, 0.4, 0.8, 1.5, 2.25, 2.5};
    // arm_service = {l_armAngleStart, -65.0, -60.0, 0.0, 70.0, l_armAngleEnd};
    // extender_service = {l_extenderPosStart, 0.25, 0.25, 0.25, 1.0,  l_extenderPosEnd};
    // wrist_service = {l_wristAngleStart, 60.0, 85.0, 85.0, 0.0, l_wristAngleEnd}; 
}

void Arm::RearPickup_To_Mid()
{
    double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    { 
    case CONE:
        l_armAngleStart = CONE_ARM_REARPICKUP_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_REARPICKUP_POS;
        l_wristAngleStart = CONE_WRIST_REARPICKUP_ANGLE;
        l_armAngleEnd = CONE_ARM_MID_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_MID_POS;
        l_wristAngleEnd = CONE_WRIST_MID_ANGLE;
        break;
    case CUBE:
        l_armAngleStart = CUBE_ARM_REARPICKUP_ANGLE;
        l_extenderPosStart = CUBE_EXTENDER_REARPICKUP_POS;
        l_wristAngleStart = CUBE_WRIST_REARPICKUP_ANGLE;
        l_armAngleEnd = CUBE_ARM_MID_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_MID_POS;
        l_wristAngleEnd = 0.0; 
        break;
    default:
        l_armAngleStart = CONE_ARM_REARPICKUP_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_REARPICKUP_POS;
        l_wristAngleStart = CONE_WRIST_REARPICKUP_ANGLE;
        l_armAngleEnd = CONE_ARM_MID_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_MID_POS;
        l_wristAngleEnd = CONE_WRIST_MID_ANGLE;
        break;
    }
    time_service = {0.0, 0.5, 0.8, 1.2, 1.6};
    arm_service = {l_armAngleStart, -60.0, 0.0, 65.0, l_armAngleEnd};
    extender_service = {l_extenderPosStart, 0.25, 0.25, 0.25,  l_extenderPosEnd};
    wrist_service = {l_wristAngleStart, 95.0, 95.0, 95.0, l_wristAngleEnd}; 
    // time_service = {0.0, 0.4, 0.8, 1.5, 2.25, 2.5};
    // arm_service = {l_armAngleStart, -65.0, -60.0, 0.0, 65.0, l_armAngleEnd};
    // extender_service = {l_extenderPosStart, 0.25, 0.25, 0.25, 1.0,  l_extenderPosEnd};
    // wrist_service = {l_wristAngleStart, 60.0, 85.0, 85.0, 0.0, l_wristAngleEnd}; 
}
void Arm::RearPickup_To_Low()
{
    double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    { 
    case CONE:
        l_armAngleStart = CONE_ARM_REARPICKUP_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_REARPICKUP_POS;
        l_wristAngleStart = CONE_WRIST_REARPICKUP_ANGLE;
        l_armAngleEnd = CONE_ARM_LOW_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_LOW_POS;
        l_wristAngleEnd = CONE_WRIST_LOW_ANGLE;
        break;
    case CUBE:
        l_armAngleStart = CUBE_ARM_REARPICKUP_ANGLE;
        l_extenderPosStart = CUBE_EXTENDER_REARPICKUP_POS;
        l_wristAngleStart = CUBE_WRIST_REARPICKUP_ANGLE;
        l_armAngleEnd = CUBE_ARM_LOW_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_LOW_POS;
        l_wristAngleEnd = CUBE_WRIST_LOW_ANGLE; 
        break;
    default:
        l_armAngleStart = CONE_ARM_REARPICKUP_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_REARPICKUP_POS;
        l_wristAngleStart = CONE_WRIST_REARPICKUP_ANGLE;
        l_armAngleEnd = CONE_ARM_LOW_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_LOW_POS;
        l_wristAngleEnd = CONE_WRIST_LOW_ANGLE;
        break;
    }
    time_service = {0.0, 0.5, 0.8, 1.2, 1.5};
    arm_service = {l_armAngleStart, -60.0, 0.0, 15.0, l_armAngleEnd};
    extender_service = {l_extenderPosStart, 0.25, 0.25, 0.25,  l_extenderPosEnd};
    wrist_service = {l_wristAngleStart, 95.0, 95.0, 85.0, l_wristAngleEnd}; 
    // time_service = {0.0, 0.4, 0.8, 1.5, 2.25, 2.5};
    // arm_service = {l_armAngleStart, -65.0, -60.0, 0.0, 5.0, l_armAngleEnd};
    // extender_service = {l_extenderPosStart, 0.25, 0.25, 0.25, 0.25, l_extenderPosEnd};
    // wrist_service = {l_wristAngleStart, 60.0, 85.0, 85.0, 80.0, l_wristAngleEnd};
}

void Arm::High_To_RearPickup()
{
    double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    { 
    case CONE:
        l_armAngleStart = CONE_ARM_HIGH_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_HIGH_POS;
        l_wristAngleStart = CONE_WRIST_HIGH_ANGLE;
        l_armAngleEnd = CONE_ARM_REARPICKUP_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_REARPICKUP_POS;
        l_wristAngleEnd = CONE_WRIST_REARPICKUP_ANGLE;
        break;
    case CUBE:
        l_armAngleStart = CUBE_ARM_HIGH_ANGLE;
        l_extenderPosStart = CUBE_EXTENDER_HIGH_POS;
        l_wristAngleStart = 0.0;
        l_armAngleEnd = CUBE_ARM_REARPICKUP_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_REARPICKUP_POS;
        l_wristAngleEnd = CUBE_WRIST_REARPICKUP_ANGLE;
        break;
    default:
        l_armAngleStart = CONE_ARM_HIGH_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_HIGH_POS;
        l_wristAngleStart = CONE_WRIST_HIGH_ANGLE;
        l_armAngleEnd = CONE_ARM_REARPICKUP_ANGLE;
        l_extenderPosEnd = CONE_EXTENDER_REARPICKUP_POS;
        l_wristAngleEnd = CONE_WRIST_REARPICKUP_ANGLE;
        break;
    }
    time_service = {0.0, 0.3, 1.1, 1.6, 1.9};
    arm_service = {l_armAngleStart, 70.0, 0.0, -44.0, l_armAngleEnd};
    extender_service = {l_extenderPosStart, 0.25, 0.25, 0.25,  l_extenderPosEnd}; 
    wrist_service = {l_wristAngleStart, 35.0, -40.0, -40.0, l_wristAngleEnd}; 
}

//! I based all services off of function corresponding to base mode and just changed constants as needed 
void Arm::Stow_To_Mid_Nose_Mode()
{
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleEnd = CONE_ARM_MID_ANGLE_NOSEMODE;
        l_extenderPosEnd = CONE_EXTENDER_MID_POS_NOSEMODE;
        l_wristAngleEnd = CONE_WRIST_MID_ANGLE_NOSEMODE;
        break;
    case CUBE:
        l_armAngleEnd = CUBE_ARM_MID_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_MID_POS;
        l_wristAngleEnd = 0.0;
        break;
    default:
        l_armAngleEnd = CONE_ARM_MID_ANGLE_NOSEMODE;
        l_extenderPosEnd = CONE_EXTENDER_MID_POS_NOSEMODE;
        l_wristAngleEnd = CONE_WRIST_MID_ANGLE_NOSEMODE;
        break;
    }
    time_service = {0.0, 0.5, 1.0};
    arm_service = {CONE_ARM_STOW_ANGLE, 65.0, l_armAngleEnd};
    extender_service = {CONE_EXTENDER_STOW_POS, 0.25, l_extenderPosEnd};
    wrist_service = {CONE_WRIST_STOW_ANGLE, 20.0, l_wristAngleEnd};   
}
void Arm::Stow_To_High_Nose_Mode()
{
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleEnd = CONE_ARM_HIGH_ANGLE_NOSEMODE;
        l_extenderPosEnd = CONE_EXTENDER_HIGH_POS_NOSEMODE;
        l_wristAngleEnd = CONE_WRIST_HIGH_ANGLE_NOSEMODE;
        break;
    case CUBE:
        l_armAngleEnd = CUBE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = 0.0;
        break;
    default:
        l_armAngleEnd = CONE_ARM_HIGH_ANGLE_NOSEMODE;
        l_extenderPosEnd = CONE_EXTENDER_HIGH_POS_NOSEMODE;
        l_wristAngleEnd = CONE_WRIST_HIGH_ANGLE_NOSEMODE;
        break;
    }
    time_service = {0.0, 0.75, 1.0};
    arm_service = {CONE_ARM_STOW_ANGLE, 83.0, l_armAngleEnd};
    extender_service = {CONE_EXTENDER_STOW_POS, 1.0, l_extenderPosEnd};
    wrist_service = {CONE_WRIST_STOW_ANGLE, 0.0, l_wristAngleEnd};
}
void Arm::Low_To_Mid_Nose_Mode()
{
    double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleStart = CONE_ARM_LOW_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_LOW_POS;
        l_wristAngleStart = CONE_WRIST_LOW_ANGLE;
        l_armAngleEnd = CONE_ARM_MID_ANGLE_NOSEMODE;
        l_extenderPosEnd = CONE_EXTENDER_MID_POS_NOSEMODE;
        l_wristAngleEnd = CONE_WRIST_MID_ANGLE_NOSEMODE;
        break;
    case CUBE:
        l_armAngleStart = CUBE_ARM_LOW_ANGLE;
        l_extenderPosStart = CUBE_EXTENDER_LOW_POS;
        l_wristAngleStart = CUBE_WRIST_LOW_ANGLE;
        l_armAngleEnd = CUBE_ARM_MID_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_MID_POS;
        l_wristAngleEnd = 0.0;
        break;
    default:
        l_armAngleStart = CONE_ARM_LOW_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_LOW_POS;
        l_wristAngleStart = CONE_WRIST_LOW_ANGLE;
        l_armAngleEnd = CONE_ARM_MID_ANGLE_NOSEMODE;
        l_extenderPosEnd = CONE_EXTENDER_MID_POS_NOSEMODE;
        l_wristAngleEnd = CONE_WRIST_MID_ANGLE_NOSEMODE;
        break;
    }
    time_service = {0.0, 0.5};
    arm_service = {l_armAngleStart, l_armAngleEnd};
    extender_service = {l_extenderPosStart, l_extenderPosEnd};
    wrist_service = {l_wristAngleStart, l_wristAngleEnd};
}
void Arm::Low_To_High_Nose_Mode()
{
    double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleStart = CONE_ARM_LOW_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_LOW_POS;
        l_wristAngleStart = CONE_WRIST_LOW_ANGLE;
        l_armAngleEnd = CONE_ARM_HIGH_ANGLE_NOSEMODE;
        l_extenderPosEnd = CONE_EXTENDER_HIGH_POS_NOSEMODE;
        l_wristAngleEnd = CONE_WRIST_HIGH_ANGLE_NOSEMODE;
        break;
    case CUBE:
        l_armAngleStart = CUBE_ARM_LOW_ANGLE;
        l_extenderPosStart = CUBE_EXTENDER_LOW_POS;
        l_wristAngleStart = CUBE_WRIST_LOW_ANGLE;
        l_armAngleEnd = CUBE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = 0.0;
        break;
    default:
        l_armAngleStart = CONE_ARM_LOW_ANGLE;
        l_extenderPosStart = CONE_EXTENDER_LOW_POS;
        l_wristAngleStart = CONE_WRIST_LOW_ANGLE;
        l_armAngleEnd = CONE_ARM_HIGH_ANGLE_NOSEMODE;
        l_extenderPosEnd = CONE_EXTENDER_HIGH_POS_NOSEMODE;
        l_wristAngleEnd = CONE_WRIST_HIGH_ANGLE_NOSEMODE;
        break;
    }
    time_service = {0.0, 0.5};
    arm_service = {l_armAngleStart, l_armAngleEnd};
    extender_service = {l_extenderPosStart, l_extenderPosEnd};
    wrist_service = {l_wristAngleStart, l_wristAngleEnd};
}
void Arm::Mid_To_High_Nose_Mode()
{
    double l_armAngleStart, l_extenderPosStart, l_wristAngleStart;
    double l_armAngleEnd, l_extenderPosEnd, l_wristAngleEnd;
    Gamepiece l_gamepiece = mainRobot->m_grabber->GetGamepiece();
    switch (l_gamepiece)
    {
    case CONE:
        l_armAngleStart = CONE_ARM_MID_ANGLE_NOSEMODE;
        l_extenderPosStart = CONE_EXTENDER_MID_POS_NOSEMODE;
        l_wristAngleStart = CONE_WRIST_MID_ANGLE_NOSEMODE;
        l_armAngleEnd = CONE_ARM_HIGH_ANGLE_NOSEMODE;
        l_extenderPosEnd = CONE_EXTENDER_HIGH_POS_NOSEMODE;
        l_wristAngleEnd = CONE_WRIST_HIGH_ANGLE_NOSEMODE;
        break;
    case CUBE:
        l_armAngleStart = CUBE_ARM_MID_ANGLE;
        l_extenderPosStart = CUBE_EXTENDER_MID_POS;
        l_wristAngleStart = 0.0;
        l_armAngleEnd = CUBE_ARM_HIGH_ANGLE;
        l_extenderPosEnd = CUBE_EXTENDER_HIGH_POS;
        l_wristAngleEnd = 0.0;
        break;
    default:
        l_armAngleStart = CONE_ARM_MID_ANGLE_NOSEMODE;
        l_extenderPosStart = CONE_EXTENDER_MID_POS_NOSEMODE;
        l_wristAngleStart = CONE_WRIST_MID_ANGLE_NOSEMODE;
        l_armAngleEnd = CONE_ARM_HIGH_ANGLE_NOSEMODE;
        l_extenderPosEnd = CONE_EXTENDER_HIGH_POS_NOSEMODE;
        l_wristAngleEnd = CONE_WRIST_HIGH_ANGLE_NOSEMODE;
        break;
    }
    time_service = {0.0, 0.5};
    arm_service = {l_armAngleStart, l_armAngleEnd};
    extender_service = {l_extenderPosStart, l_extenderPosEnd};
    wrist_service = {l_wristAngleStart, l_wristAngleEnd};
}


void Arm::InversePos()
{
    std::vector<double> temp_time_service;

    if (time_service.size() < 1)
        return;

    temp_time_service.push_back(0.0);

    for (int i = time_service.size() - 1; i > 0; i--)
    {
        temp_time_service.push_back(time_service[i] - time_service[i - 1]);
    }
    std::reverse(arm_service.begin(), arm_service.end());
    std::reverse(extender_service.begin(), extender_service.end());
    std::reverse(wrist_service.begin(), wrist_service.end());
}
void Arm::MotionService()
{
    switch (serviceStage)
    {
    case 0:
        service_offset = mainRobot->GetLoopCount();
        service_idx = 1;
        serviceStage++;
        break;
    case 1:
        double time = (mainRobot->GetLoopCount() - service_offset) * LOOPTIME;
        if (time > (time_service[service_idx] + LOOPTIME/2.0))
        {
            service_idx++;
        }
        if (service_idx >= time_service.size())
        {
            service_idx = 1;
            serviceStage = 0;

            isMotionService = false;
            break;
        }
        double k, l_armPos, l_extenderPos, l_wristPos;
        k = (time_service[service_idx] - time) / (time_service[service_idx] - time_service[service_idx - 1]);
        l_armPos = k * arm_service[service_idx - 1] + arm_service[service_idx] * (1.0 - k);
        l_extenderPos = k * extender_service[service_idx - 1] + extender_service[service_idx] * (1.0 - k);
        l_wristPos = k * wrist_service[service_idx - 1] + wrist_service[service_idx] * (1.0 - k);

        // printf("arm angle, extender pos, wrist angle\n");
        // printf("%f, %f, %f\n", l_armPos, l_extenderPos, l_wristPos);
        SetArmAngle(l_armPos);
        SetExtenderPos(l_extenderPos);
        mainRobot->m_grabber->SetWristAngle(l_wristPos);
        break;
    }
}

void Arm::Service()
{

    if (isMotionService)
    {
        MotionService();
    }

    if (!isExtenderCal)
    {
        ExtenderCalibrate();
        armMotor->Set(0.0);
    }
    else
    {
        ArmControl();
    }

    ExtenderControl();
}

void Arm::UpdateDash()
{
    std::string position_name;
    position_name = "not set";
    switch (position)
    {
    case STOW_POS:
        position_name = "Stow Pos";
        break;

    case PICKUP_POS:
        position_name = "Pickup Pos";
        break;

    case HIGH_POS:
        position_name = "High Pos";
        break;

    case MID_POS:
        position_name = "Mid Pos";
        break;

    case LOW_POS:
        position_name = "Low Pos";
        break;
    case SUB_POS:
        position_name = "Sub Pos";
        break;
    case REARPICKUP_POS:
        position_name = "Rear Pickup Pos";
        break;
    }
    frc::SmartDashboard::PutString("Current Position:", position_name);
    frc::SmartDashboard::PutNumber("Bias Cmd", armPosCmdBias);
    frc::SmartDashboard::PutNumber("Arm Angle", armTheta);
    frc::SmartDashboard::PutNumber("Extender Pos", extenderPos);
    frc::SmartDashboard::PutNumber("Arm Motor Cmd", armMotorCmd);
    frc::SmartDashboard::PutNumber("Arm Vel Cmd", armVelCmd);
    frc::SmartDashboard::PutNumber("Raw Arm Theta", rawArmTheta);
    frc::SmartDashboard::PutNumber("Nose Mode", noseMode);
}