#include "Common.h"
#include "TalonXXV.h"
#include "Grabber.h"

Grabber::Grabber(TalonXXV *pRobot)
{
    mainRobot = pRobot;

    // absolute encoder for wrist
    wristEncoderInput = new frc::DigitalInput(WRIST_ENCODER);
    wristEncoder = new frc::DutyCycle(wristEncoderInput);

    wristCylinder = new frc::DoubleSolenoid(PCM_PNEUMATICS_MODULE, frc::PneumaticsModuleType::CTREPCM, PCM_WRIST_1, PCM_WRIST_2);

    wristMotor = new frc::PWMTalonSRX(PWM_WRIST_MOTOR);
    rollerMotor = new frc::PWMTalonSRX(PWM_ROLLER_MOTOR);

    intakeBeamBreak = new frc::DigitalInput(INTAKE_BEAM_BREAK);

    StartingConfig();
}

void Grabber::LocalReset()
{
    wristAngleCmd = 0.0;
    wristVelCmd = 0.0;
    wristVelErr = 0.0;
    wristAcc = 0.0;
    wristMotorCmd = 0.0;
    wristPosErr = 0.0;
    wristErrInt = 0.0;

    rollerCmd = 0.0;
    intakeState = 0;
    isGamepieceLoaded = false;

    holdGamepiece = false;
    rollerCurrentF = 0.0;
    gamepiece = NONE;

    beamBreakLoopcount = 0;

    GatherReset();
}

void Grabber::StartingConfig()
{
    wristAngleBias = 0.0;

    LocalReset();
    //GatherCone();
}

void Grabber::StopAll()
{
    LocalReset();
}

/**
 * @brief sets an angle for the wrist
 * @param angle wrist angle cmd in deg
 */
void Grabber::SetWristAngle(double angle)
{
    wristAngleCmd = angle;
}

/**
 * @brief gathers a cone gamepiece: uses hold cmd if cone already collected, otherwise uses intake cmd
 */
void Grabber::GatherCone()
{
    gamepiece=CONE;

    wristCylinder->Set(frc::DoubleSolenoid::Value::kForward);
    if (isBeamBroken || isStalled || holdGamepiece)
    {
        rollerCmd = CONE_INTAKE_HOLD_CMD;
        holdGamepiece = true;
        mainRobot->m_ledPanel->DisplayBlank();
    }
    else
    {
        rollerCmd = CONE_INTAKE_ROLLER_CMD;
    }
}

/**
 * @brief gathers a cube gamepiece: uses hold cmd if cube already collected, otherwise uses intake cmd
 */
void Grabber::GatherCube()
{
    gamepiece=CUBE;

    wristCylinder->Set(frc::DoubleSolenoid::Value::kReverse);
    if (isBeamBroken || holdGamepiece)
    {
        rollerCmd = CUBE_INTAKE_HOLD_CMD;
        holdGamepiece = true;
        mainRobot->m_ledPanel->DisplayBlank();
    }
    else
    {
        rollerCmd = CUBE_INTAKE_ROLLER_CMD;
    }
}



/**
 * @brief eject cone with eject cmd
 */
void Grabber::EjectCone()
{
    rollerCmd = CONE_EJECT_ROLLER_CMD;
    holdGamepiece = false;
    gamepiece=NONE;
}

void Grabber::EjectConeFaster()
{
    rollerCmd = CONE_EJECT_ROLLER_FULL_CMD;
    holdGamepiece = false;
    gamepiece = NONE;
}

/**
 * @brief eject cube with eject cmd
 */
void Grabber::EjectCube()
{
    rollerCmd = CUBE_EJECT_ROLLER_CMD;
    holdGamepiece = false;
    gamepiece=NONE;
}

void Grabber::EjectCubeFaster()
{
    rollerCmd = CUBE_EJECT_ROLLER_FULL_CMD;
    holdGamepiece = false;
    gamepiece = NONE;
}

void Grabber::UpdateDash()
{
    // printf("Wrist Encoder: %f\n", wristEncoder->GetOutput());
    frc::SmartDashboard::PutNumber("wrist motor cmd", wristMotorCmd);
    frc::SmartDashboard::PutNumber("wrist theta", wristTheta);
    frc::SmartDashboard::PutNumber("raw wrist theta", rawWristTheta);
    frc::SmartDashboard::PutNumber("roller current", rollerCurrent);
    frc::SmartDashboard::PutNumber("is beam broken", isBeamBroken);
    frc::SmartDashboard::PutNumber("is stalled", isStalled);
    frc::SmartDashboard::PutNumber("roller motor cmd", rollerCmd);
    frc::SmartDashboard::PutNumber("hold gamepiece", holdGamepiece);
    frc::SmartDashboard::PutNumber("Gamepiece", gamepiece);
}

void Grabber::Analyze()
{
    // calculations for arm
    lastWristTheta = wristTheta;
    rawWristTheta = 360.0 * (wristEncoder->GetOutput() - DMIN) / (DMAX - DMIN);
    wristTheta = -(rawWristTheta - WRIST_ENCODER_OFFSET + wristAngleBias); // encoder reads backwards
    wristTheta = TalonXXV::Wrap(wristTheta);
    // if (wristTheta > 180.0)
    // {
    //     wristTheta -= 360.0;
    // }
    // else if (wristTheta < -180.0)
    // {
    //     wristTheta += 360;
    // }

    wristVel = (wristTheta - lastWristTheta) / LOOPTIME;
    wristVel = TalonXXV::Limit(-200.0, 200.0, wristVel);

    rollerCurrent = mainRobot->m_pdp->GetCurrent(ROLLER_CURRENT_IN);
    rollerCurrentF = rollerCurrentF + (rollerCurrent - rollerCurrentF)*10.0*LOOPTIME;

    isStalled = (rollerCurrentF >= GRABBER_STALL);

    // isBeamBroken = !(intakeBeamBreak->Get());

    if(!(intakeBeamBreak->Get()))
    {
        beamBreakLoopcount++;
    }
    else
    {
        beamBreakLoopcount = 0;
    }
    if(beamBreakLoopcount >= (round(0.04/LOOPTIME))) // old val was 0.01
    {
        isBeamBroken = true;
    }
    else
    {
        isBeamBroken = false;
    }

}

void Grabber::Service()
{
    // wristAngleCmd = 90.0 * (mainRobot->m_userInput->GetFlightCtrl_RAW_Z());
    wristPosErr = wristAngleCmd - wristTheta;
    wristVelCmd = wristPosErr * WRIST_K_POS;
    // wristVelCmd = 60.0*(mainRobot->m_userInput->GetFlightCtrl_RAW_Z());
    wristVelErr = wristVelCmd - wristVel;
    wristErrInt = wristErrInt + wristVelErr * LOOPTIME;
    wristErrInt = TalonXXV::Limit(-WRIST_K_ROBOT / WRIST_K_VEL, WRIST_K_ROBOT / WRIST_K_VEL, wristErrInt);
    wristMotorCmd = WRIST_K_VEL / WRIST_K_ROBOT * (WRIST_TAU_ROBOT * wristVelErr + wristErrInt);
    wristMotorCmd = TalonXXV::Limit(-1.0, 1.0, wristMotorCmd);

    // wristMotorCmd = (mainRobot->m_userInput->GetFlightCtrl_RAW_Z());

    wristMotor->Set(-wristMotorCmd);
    // if (mainRobot->GetLoopCount() % 50 == 0)
    // {
    //     // printf("cmd,theta,vel,rawtheta %f, %f, %f, %f\n", wristMotorCmd, wristTheta, wristVel, rawWristTheta);
    //     printf("roller current: %f, beam broken: %d\n", rollerCurrent, isBeamBroken);

    // }

    // wristCylinder->Set(wristCylinderPos);

    rollerMotor->Set(rollerCmd);
}
