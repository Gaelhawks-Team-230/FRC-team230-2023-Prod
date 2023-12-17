#include "frc/RobotController.h"
#include "Common.h"
#include "SwerveModule.h"
#include "TalonXXV.h"

using namespace std;

/**
 * @brief Constructor for a single swerve module
 *
 * @param pRobot main robot
 * @param steerFalconID CAN ID of steer falcon
 * @param driveFalconID CAN ID of drive falcon
 * @param absoluteEncoderID CAN ID of the CANCoder ID
 * @return SwerveModule
 */
SwerveModule::SwerveModule(TalonXXV *pRobot, unsigned int steerFalconID, unsigned int driveFalconID, unsigned int absoluteEncoderID, unsigned int swerveID)
{

    mainRobot = pRobot;
    m_moduleID = swerveID;

    m_steer = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(steerFalconID);
    m_drive = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(driveFalconID);
    m_steerEncoder = new ctre::phoenix::sensors::CANCoder(absoluteEncoderID);

    StartingConfig();
    LocalReset();
}
/**
 * @brief Set Falcons to starting configuration
 *
 */
void SwerveModule::StartingConfig()
{
#ifdef CTRE_FACTORY_RESET
    printf("Swerve Module Factory Reset\n");
    m_steer->ConfigFactoryDefault();
    m_drive->ConfigFactoryDefault();
    m_steerEncoder->ConfigFactoryDefault();
#endif

    m_steer->SetNeutralMode(NeutralMode::Brake);
    m_drive->SetNeutralMode(NeutralMode::Brake);

    m_drive->ConfigVelocityMeasurementPeriod(SensorVelocityMeasPeriod::Period_50Ms);
    m_drive->ConfigVelocityMeasurementWindow(VELOCITY_MEAS_WINDOW);

    m_steer->ConfigVelocityMeasurementPeriod(SensorVelocityMeasPeriod::Period_50Ms);
    m_steer->ConfigVelocityMeasurementWindow(VELOCITY_MEAS_WINDOW);

    m_drive->ConfigNeutralDeadband(DRIVE_DEADBAND);
    m_steer->ConfigNeutralDeadband(STEER_DEADBAND);
}
/**
 * @brief Reset all variabes to default
 *
 */
void SwerveModule::LocalReset()
{
    m_driveLastPosition = m_drive->GetSelectedSensorPosition();

    m_driveLastPosition = 0.0;
    m_driveCurrentPosition = 0.0;
    m_driveRaw = 0.0;
    m_driveOffset = 0.0;
    m_drivePosition = 0.0;
    m_driveVel = 0.0;
    m_driveMotorVel = 0.0;
    m_drivePosVel = 0.0;

    m_steerMotorPosition = 0.0;
    m_steerMotorVel = 0.0;
    m_steerEncoderPosition = 0.0;
    m_steerEncoderAbsolutePosition = 0.0;
    m_steerEncoderVel = 0.0;

    m_steer_pErr = 0.0;
    m_steer_vCmd = 0.0;
    m_steer_vErr = 0.0;
    m_steer_vErrIntegrator = 0.0;
    m_steer_cmd = 0.0;

    m_rev = 0.0;

    m_drive_pErr = 0.0;
    m_drive_vCmd = 0.0;
    m_drive_vErr = 0.0;
    m_drive_vErrIntegrator = 0.0;
    m_drive_cmd = 0.0;
}
/**
 * @brief Stop all motor functions and reset variables
 *
 */
void SwerveModule::StopAll()
{
    m_drive->StopMotor();
    m_steer->StopMotor();
    LocalReset();
}
/**
 * @brief Set drive falcon cmd
 *
 * @param cmd -1 to 1 falcon cmd
 * @note cmd is negated as motor direction is wrong
 */
void SwerveModule::SetDriveCmd(double cmd)
{
    // * NOTE cmd was negative on PoC swerve
    m_drive->Set(-cmd);
}
/**
 * @brief Set steer falcon cmd
 *
 * @param cmd -1 to 1 falcon cmd
 * @note cmd is negated as motor direction is wrong
 */
void SwerveModule::SetWheelCmd(double cmd)
{
    // * NOTE cmd was negative on PoC swerve
    m_steer->Set(cmd);
}
/**
 * @brief Update dashboard with swerve module data
 *
 */
void SwerveModule::UpdateDash()
{
    string l_moduleName;
    l_moduleName = "Module " + to_string(m_moduleID) + ": ";

    frc::SmartDashboard::PutNumber(l_moduleName + "Drive raw position: ", m_drivePosition);
    frc::SmartDashboard::PutNumber(l_moduleName + "Drive velocity (inches per second): ", m_driveVel);
    // frc::SmartDashboard::PutNumber(l_moduleName + "Drive position velocity (inches per second): ", m_drivePosVel);
    frc::SmartDashboard::PutNumber(l_moduleName + "Drive cntrl vel (inches per second): ", drivecntrl_vel);
    frc::SmartDashboard::PutNumber(l_moduleName + "Drive cntrl angle (degrees per second): ", drivecmtrl_angle);
    // frc::SmartDashboard::PutNumber(l_moduleName + "Steer motor raw position: ", m_steerMotorPosition);
    // frc::SmartDashboard::PutNumber(l_moduleName + "Steer encoder position: ", m_steerEncoderPosition);
    frc::SmartDashboard::PutNumber(l_moduleName + "Steer encoder absolute position: ", m_steerEncoderAbsolutePosition);
    // loopcount, drive position, drive motor velocity, steer motor position, steer motor velocity, steer encoder absolute position, steer encoder position, steer encoder velocity, can pct utilization
    // printf("%f, %f, %f, %f, %f, %f[], %f, %f, %f\n",mainRobot->GetLoopCount(), m_drivePosition, m_driveMotorVel,m_steerMotorPosition, m_steerMotorVel,m_steerEncoderAbsolutePosition, m_steerEncoderPosition,m_steerEncoderVel,(frc::RobotController::GetCANStatus().percentBusUtilization*100));
}
/**
 * @brief Collect sensor states
 * @note Negated sensor values according to correct motor direction
 *
 */
void SwerveModule::Analyze()
{
    // * Note pos and vel was negative on PoC swerve
    m_drivePosition = -m_drive->GetSelectedSensorPosition();
    m_driveMotorVel = -m_drive->GetSelectedSensorVelocity();

    m_drivePosVel = (m_drivePosition - m_driveLastPosition) * GetPositionVelocityScaleFactor();
    m_driveVel = m_driveMotorVel * GetVelocityScaleFactor();
    m_driveLastPosition = m_drivePosition;

    // ? Check steer motor direction
    m_steerMotorPosition = -m_steer->GetSelectedSensorPosition();
    m_steerMotorVel = -m_steer->GetSelectedSensorVelocity();

    // adjust abs sensor range from 0-360 to -180 to 180
    // ? Check steer encoder direction
    m_steerEncoderAbsolutePosition = -m_steerEncoder->GetAbsolutePosition() + 180.0;

    // ? Check steer encoder velocty
    m_steerEncoderPosition = -m_steerEncoder->GetPosition();
    m_steerEncoderVel = -m_steerEncoder->GetVelocity();
}

/**
 * @brief Control swerve module
 *
 * @param vel velocity in inches per second
 * @param angle angle cmd from -180 to 180
 */
void SwerveModule::DriveControl(double vel, double angle)
{
    drivecmtrl_angle = angle;
    drivecntrl_vel = vel;
    m_steer_pErr = angle - m_steerEncoderAbsolutePosition + 180.0 * m_rev;
    if (fabs(TalonXXV::Wrap(m_steer_pErr)) > 90.0)
    {
        m_rev = !m_rev;
        m_steer_pErr += 180.0;
    }
    m_steer_pErr = TalonXXV::Wrap(m_steer_pErr);
    m_steer_vCmd = STEER_kP * m_steer_pErr;
    m_steer_vErr = m_steer_vCmd - m_steerEncoderVel;
    m_steer_vErrIntegrator += m_steer_vErr * LOOPTIME;
    m_steer_vErrIntegrator = TalonXXV::Limit(-STEER_kR / STEER_kV, STEER_kR / STEER_kV, m_steer_vErrIntegrator);
    m_steer_cmd = (STEER_kV / STEER_kR) * (m_steer_vErr * STEER_TAU + m_steer_vErrIntegrator);

    if (m_rev)
    {
        vel *= -1.0;
    }
    SetWheelCmd(m_steer_cmd);

    m_drive_vErr = vel - m_driveVel + kAV * m_steer_vCmd;
    m_drive_vErrIntegrator += m_drive_vErr * LOOPTIME;
    m_drive_vErrIntegrator = TalonXXV::Limit(-DRIVE_kR / DRIVE_kV, DRIVE_kR / DRIVE_kV, m_drive_vErrIntegrator);
    m_drive_cmd = (DRIVE_kV / DRIVE_kR) * (m_drive_vErr * DRIVE_TAU + m_drive_vErrIntegrator);

    SetDriveCmd(m_drive_cmd);
}