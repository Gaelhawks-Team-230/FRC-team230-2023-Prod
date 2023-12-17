#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/AnalogGyro.h>
#include "Common.h"
#include "SwerveModule.h"

// Measured in inches, used in code as inches
// Point 1 coordinates:
const double P1X = (29.375);
const double P1Y = (22.375);
// Point 2 coordinates:
const double P2X = (2.625);
const double P2Y = (22.375);
// Point 3 coordinates:
const double P3X = (2.625);
const double P3Y = (2.625);
// Point 4 coordinates:
const double P4X = (29.375);
const double P4Y = (2.625);
// Center coordinates (from bottom left corner):
const double CX = (16.0);
const double CY = (12.5);
// Offset of theta for each module
// old data offsets pre 3/37
//  m1 -36.4
//  m2 153.63
//  m3 -0.8788
//  m4 147.65
//  offsets for 3/27
//  m1 -36.73
//  m2 153.28
//  m3 -0.26
//  m4 -165.75


const double P1Offset = -36.65;
const double P2Offset = 153.54;
const double P3Offset = -0.61;
const double P4Offset = 144.05;
const double PStart = 0;

#define GYRO_MAX_VEL (360.0) // degrees per second
#define ROBOT_ORR_GYRO_OFFSET (180.0)

#define PITCH_GYRO_SF (90.0/41.0)
#define KT (0.35)
#define KW (0.35) 

#define R_kV (5.0)
#define R_kR (1.0)
#define R_TAU (0.075)
#define R_FF (0.0)

class TalonXXV;

class Drivetrain
{

private:
    TalonXXV *mainRobot;

    SwerveModule *m_frontLeftModule;
    SwerveModule *m_frontRightModule;
    SwerveModule *m_backLeftModule;
    SwerveModule *m_backRightModule;

    frc::ADXRS450_Gyro *m_gyro;
    frc::AnalogGyro *m_pitchGyro;

    double m_gyroPitchReading;
    double m_gyroPitchVel;
    double m_gyroPitchLastReading;

    double m_gyroReading;
    double m_gyroVel;
    double m_gyroLastReading;
    bool m_zeroVelDebugMode;
    bool m_gyroEnabled;

    double m_gyro_offset;

    double m_xf;
    double m_yf;
    double m_zf;

    // gyro stuff
    double m_r_vCmd;
    double m_r_vErr;
    double m_r_vErrIntegrator;
    double m_r_cmd;
    double m_r_ff;


    double angle_count;
    double pitchMax;

    bool m_Balancing;
    unsigned int m_balanceStage;


public:
    Drivetrain(TalonXXV *pRobot);
    void LocalReset(void);
    void StopAll(void);
    void UpdateDash(void);
    void DriveControl(double xdot, double ydot, double psidot);
    void Analyze(void);
    void GyroReset(void);
    void PitchGyroReset(void);
    double GetGyroReading(void) { return m_gyroReading; };
    double GetGyroVel(void) { return m_gyroVel; };
    void SetZeroVelDebugModeOn() { m_zeroVelDebugMode = true; };
    void SetZeroVelDebugModeOff() { m_zeroVelDebugMode = false; };
    void CmdModel(double x, double y, double z,
                  double *pxout, double *pyout, double *pzout,
                  double kx, double ky, double kz);

    void SwerveKinematics(double xdot, double ydot, double psidot,
                          double &v1c, double &v2c, double &v3c, double &v4c,
                          double &t1c, double &t2c, double &t3c, double &t4c);
    void ToFieldCoordinates(double *xdot, double *ydot);
    void AutoBalance(double *xdot);
    void EnableGyro(){ m_gyroEnabled=true;};
    void DisableGyro(){ m_gyroEnabled=false;};
    double GetGyroPitchReading(){return m_gyroPitchReading;};
    double GetGyroPitchVel(){return m_gyroPitchVel;};
    double GetGyroPitchMax(){return pitchMax;};

};