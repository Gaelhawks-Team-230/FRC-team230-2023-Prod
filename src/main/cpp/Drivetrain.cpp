#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include "Common.h"
#include "Drivetrain.h"
#include "TalonXXV.h"
/**
 * @brief Construct a new Drivetrain object
 *
 * @param pRobot pointer to the main robot class
 */
Drivetrain::Drivetrain(TalonXXV *pRobot)
{
    mainRobot = pRobot;

    m_frontLeftModule = new SwerveModule(mainRobot, FRONT_LEFT_STEER, FRONT_LEFT_DRIVE, FRONT_LEFT_ABSOLUTE_ENCODER, 4);
    m_frontRightModule = new SwerveModule(mainRobot, FRONT_RIGHT_STEER, FRONT_RIGHT_DRIVE, FRONT_RIGHT_ABSOLUTE_ENCODER, 1);
    m_backLeftModule = new SwerveModule(mainRobot, BACK_LEFT_STEER, BACK_LEFT_DRIVE, BACK_LEFT_ABSOLUTE_ENCODER, 3);
    m_backRightModule = new SwerveModule(mainRobot, BACK_RIGHT_STEER, BACK_RIGHT_DRIVE, BACK_RIGHT_ABSOLUTE_ENCODER, 2);

    m_gyro = new frc::ADXRS450_Gyro();
    m_gyro->Calibrate();

    m_pitchGyro = new frc::AnalogGyro(PITCH_GYRO);
    // calling calibrate after init calibrates twice
    // m_pitchGyro->Calibrate();

    LocalReset();
}
/**
 * @brief Reset variables and swerve submodules
 *
 */
void Drivetrain::LocalReset()
{
    m_zeroVelDebugMode = false;

    m_frontLeftModule->LocalReset();
    m_frontRightModule->LocalReset();
    m_backLeftModule->LocalReset();
    m_backRightModule->LocalReset();

    m_gyroEnabled = false;

    m_r_vCmd = 0.0;
    m_r_vErr = 0.0;
    m_r_vErrIntegrator = 0.0;
    m_r_cmd = 0.0;
    m_gyroVel = 0.0;
    m_r_ff = 0.0;

    m_xf = 0.0;
    m_yf = 0.0;
    m_zf = 0.0;

    m_gyro_offset = ROBOT_ORR_GYRO_OFFSET;

    m_Balancing = false;
    m_balanceStage = 0;
}
/**
 * @brief Computes swerve velocity and rotational commands
 *
 * @param xdot forward velocity from joystick
 * @param ydot right velocity from joystick
 * @param psidot rotation rate around z from joystick. coming in as degrees, converted to radians.
 *
 * @param v1c velocity command for 1st module in inches/sec
 * @param v2c velocity command for 2nd module in inches/sec
 * @param v3c velocity command for 3rd module in inches/sec
 * @param v4c velocity command for 4th module in inches/sec
 *
 * @param t1c rotational command for 1st module in deg/sec
 * @param t2c rotational command for 2nd module in deg/sec
 * @param t3c rotational command for 3rd module in deg/sec
 * @param t4c rotational command for 4th module in deg/sec
 */
void Drivetrain::SwerveKinematics(double xdot, double ydot, double psidot,
                                  double &v1c, double &v2c, double &v3c, double &v4c,
                                  double &t1c, double &t2c, double &t3c, double &t4c)
{
    double l_v1x, l_v1y, l_v2x, l_v2y, l_v3x, l_v3y, l_v4x, l_v4y;
    psidot = (psidot * M_PI / 180);

    l_v1x = xdot - (psidot * (P1Y - CY));
    l_v1y = ydot + (psidot * (P1X - CX));

    l_v2x = xdot - (psidot * (P2Y - CY));
    l_v2y = ydot + (psidot * (P2X - CX));

    l_v3x = xdot - (psidot * (P3Y - CY));
    l_v3y = ydot + (psidot * (P3X - CX));

    l_v4x = xdot - (psidot * (P4Y - CY));
    l_v4y = ydot + (psidot * (P4X - CX));

    v1c = sqrt((l_v1x * l_v1x) + (l_v1y * l_v1y));
    v2c = sqrt((l_v2x * l_v2x) + (l_v2y * l_v2y));
    v3c = sqrt((l_v3x * l_v3x) + (l_v3y * l_v3y));
    v4c = sqrt((l_v4x * l_v4x) + (l_v4y * l_v4y));

    t1c = (180 / M_PI) * atan2(l_v1y, l_v1x) + P1Offset;
    t2c = (180 / M_PI) * atan2(l_v2y, l_v2x) + P2Offset;
    t3c = (180 / M_PI) * atan2(l_v3y, l_v3x) + P3Offset;
    t4c = (180 / M_PI) * atan2(l_v4y, l_v4x) + P4Offset;

    //   if (v1y==0 && v1x==0){
    //       t1c = P1Offset - PStart;
    //   }
    //    if (v2y==0 && v2x==0){
    //       t2c = P2Offset + PStart;
    //   }
    //    if (v3y==0 && v3x==0){
    //       t3c = P3Offset - PStart;
    //   }
    //    if (v4y==0 && v4x==0){
    //       t4c = P4Offset + PStart;
    //   }

    t1c = TalonXXV::Wrap(t1c);
    t2c = TalonXXV::Wrap(t2c);
    t3c = TalonXXV::Wrap(t3c);
    t4c = TalonXXV::Wrap(t4c);
}
/**
 * @brief Read sensor values
 *
 */
void Drivetrain::Analyze()
{
    m_frontLeftModule->Analyze();
    m_frontRightModule->Analyze();
    m_backRightModule->Analyze();
    m_backLeftModule->Analyze();

    m_gyroReading = m_gyro->GetAngle();
    m_gyroVel = (m_gyroReading - m_gyroLastReading) / LOOPTIME;
    m_gyroVel = TalonXXV::Limit(-GYRO_MAX_VEL, GYRO_MAX_VEL, m_gyroVel);
    m_gyroLastReading = m_gyroReading;
    // m_gyroReading = TalonXXV::Wrap(m_gyroReading);

    m_gyroPitchReading = -m_pitchGyro->GetAngle() * PITCH_GYRO_SF;
    m_gyroPitchVel = (m_gyroPitchReading - m_gyroPitchLastReading) / LOOPTIME;
    m_gyroPitchVel = TalonXXV::Limit(-GYRO_MAX_VEL, GYRO_MAX_VEL, m_gyroPitchVel);
    m_gyroPitchLastReading = m_gyroPitchReading;
    // m_gyroPitchReading = TalonXXV::Wrap(m_gyroPitchReading);

    pitchMax = max(pitchMax, m_gyroPitchReading);

    // if (mainRobot->GetLoopCount() % 50)
    // {
    //     printf("%f\n", m_gyroPitchReading);
    // }
}
/**
 * @brief Reset gyro zero
 *
 */
void Drivetrain::GyroReset()
{
    m_gyro->Reset();
    m_gyro_offset = 0.0;
    m_gyroReading = m_gyro->GetAngle();
    m_gyroVel = 0.0;
    m_gyroLastReading = m_gyroReading;
}
/**
 * @brief Reset pitch gyro to zero
 *
 */
void Drivetrain::PitchGyroReset()
{
    m_pitchGyro->Reset();
    m_gyroPitchReading = -m_pitchGyro->GetAngle() * PITCH_GYRO_SF;
    m_gyroPitchVel = 0.0;
    m_gyroPitchLastReading = m_gyroPitchReading;
    pitchMax = 0.0;
}
/**
 * @brief Control drivetrain
 *
 * @param xdot joystick x
 * @param ydot joystick y
 * @param psidot joystick z
 */
void Drivetrain::DriveControl(double xdot, double ydot, double psidot)
{
    // gyro heading loop
    m_gyroVel = GetGyroVel();
    m_r_ff = psidot;
    m_r_vErr = psidot - m_gyroVel;
    m_r_vErrIntegrator += m_r_vErr * LOOPTIME;
    double l_bounds;
    l_bounds = (R_SFACTOR * R_kR) / R_kV;
    m_r_vErrIntegrator = TalonXXV::Limit(-l_bounds, l_bounds, m_r_vErrIntegrator);
    m_r_cmd = (R_kV / R_kR) * (R_TAU * m_r_vErr + m_r_vErrIntegrator) + (R_FF * m_r_ff);

    if (!m_gyroEnabled)
    {
        m_r_cmd = psidot;
        m_r_vErrIntegrator = 0.0;
    }

    double v1c, v2c, v3c, v4c, t1c, t2c, t3c, t4c;
    double xdotFil, ydotFil, psidotFil;
    CmdModel(xdot, ydot, m_r_cmd, &xdotFil, &ydotFil, &psidotFil, 5.0, 5.0, 5.0);
    SwerveKinematics(xdotFil, ydotFil, psidotFil, v1c, v2c, v3c, v4c, t1c, t2c, t3c, t4c);

    if (m_zeroVelDebugMode)
    {
        v1c = 0.0;
        v2c = 0.0;
        v3c = 0.0;
        v4c = 0.0;
    }
    m_frontRightModule->DriveControl(v1c, t1c);
    m_backRightModule->DriveControl(v2c, t2c);
    m_backLeftModule->DriveControl(v3c, t3c);
    m_frontLeftModule->DriveControl(v4c, t4c);

    //? LED PANEL DISPLAY FOR BALANCING 
    // if (frc::DriverStation::GetMatchTime() < 30.0)
    // {
    //     switch(m_balanceStage)
    //     {
    //         case 0: 
    //             PitchGyroReset();
    //             m_balanceStage++;
    //             break;
    //         case 1:
    //             if (m_gyroPitchReading > 5.0)
    //             {
    //                 mainRobot->m_ledPanel->DisplayBalancingPos();
    //                 m_Balancing = true;
    //             }
    //             else if (m_gyroPitchReading < -5.0)
    //             {
    //                 mainRobot->m_ledPanel->DisplayBalancingNeg();
    //                 m_Balancing = true;
    //             }
    //             else 
    //             {
    //                 if (m_Balancing)
    //                 {
    //                     mainRobot->m_ledPanel->DisplayBalanced();
    //                     m_Balancing = false;
    //                 }
    //             }
    //             break;
    //     }
    // }
}
/**
 * @brief Joystick command model filter
 *
 * @param x joystick xdot
 * @param y joystick ydot
 * @param z joystick psidot
 * @param pxout xdot out
 * @param pyout ydot out
 * @param pzout psidot out
 * @param kx x constant
 * @param ky y constant
 * @param kz z constant
 */
void Drivetrain::CmdModel(double x, double y, double z,
                          double *pxout, double *pyout, double *pzout,
                          double kx, double ky, double kz)
{
    double l_xfdot, l_yfdot, l_zfdot;

    l_xfdot = (x - m_xf) * kx;
    l_yfdot = (y - m_yf) * ky;
    l_zfdot = (z - m_zf) * kz;

    m_xf = m_xf + l_xfdot * LOOPTIME;
    m_yf = m_yf + l_yfdot * LOOPTIME;
    m_zf = m_zf + l_zfdot * LOOPTIME;

    *pxout = m_xf;
    *pyout = m_yf;
    *pzout = m_zf;
    // *pxout = x;
    // *pyout = y;
    // *pzout = z;
}
/**
 * @brief Convert robot centric coordinates to field centric coordinates
 *
 * @param *xdot pointer to joystick xdot
 * @param *ydot pointer to joystick ydot
 */
void Drivetrain::ToFieldCoordinates(double *xdot, double *ydot)
{

    double l_vNorth, l_vEast, l_theta;
    l_vNorth = *xdot;
    l_vEast = *ydot;
    // field coordinates conversion
    l_theta = (GetGyroReading() + 180.0) * (M_PI / 180.0); // added 180 deg offset to orient field coords in correct direction - needs permanent fix
    l_theta = l_theta + GetGyroVel() * (M_PI / 180.0) * 0.15;
    *xdot = cos(l_theta) * l_vNorth + sin(l_theta) * l_vEast;
    *ydot = -sin(l_theta) * l_vNorth + cos(l_theta) * l_vEast;
}

void Drivetrain::AutoBalance(double *xdot)
{
    double vCmd;
    vCmd = (mainRobot->m_drivetrain->GetGyroPitchReading() * KT + mainRobot->m_drivetrain->GetGyroPitchVel() * KW);
    vCmd = TalonXXV::Limit(-4.0, 4.0, vCmd);
    // vCmd = (fabs(mainRobot->m_drivetrain->GetGyroPitchReading())
    //      * (mainRobot->m_drivetrain->GetGyroPitchReading())*KT2 + mainRobot->m_drivetrain->GetGyroPitchVel()*KW)*(90.0/41.0);
    // printf("vCmd: %f\n", vCmd);
    *xdot += vCmd;
}

/**
 * @brief Update SmartDashboard
 *
 */
void Drivetrain::UpdateDash()
{

    m_frontRightModule->UpdateDash();
    m_frontLeftModule->UpdateDash();
    m_backLeftModule->UpdateDash();
    m_backRightModule->UpdateDash();

    frc::SmartDashboard::PutNumber("Gyro Wrapped Angle", m_gyroReading);
    frc::SmartDashboard::PutNumber("Gyro Vel", m_gyroVel);

    frc::SmartDashboard::PutNumber("Pitch Gyro Angle", m_gyroPitchReading);
    frc::SmartDashboard::PutNumber("Pitch Gyro Vel", m_gyroPitchVel);
}
/**
 * @brief Stop submodules
 *
 */
void Drivetrain::StopAll()
{
    m_frontLeftModule->StopAll();
    m_frontRightModule->StopAll();
    m_backLeftModule->StopAll();
    m_backRightModule->StopAll();
}