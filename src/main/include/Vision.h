#pragma once

#include <cmath>
#include "Common.h"
#include "PoseMatrix.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include <wpi/json.h>
#include <photonlib/PhotonCamera.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#define PHOTONVISION_DIST_SF (0.654)

#define METER_INCHES (39.3701)
#define KXY (3.0)
#define KPSI (2.0)

#define XY_VEL_LIMIT (32.0) // inches per second
#define R_VEL_LIMIT (120.0) //degrees per second

#define GOAL_X (25.0) // inches away from target
#define GOAL_Y (0.0) // inches left/right from target
#define GOAL_R (180.0) // degrees from parallel

#define HEADING_OFFSET (0.0)
using namespace std;

struct Dist2Goal
{
    Vect d2g_xyz;
    Vect d2g_ned;
    double psi2g_target;
    double heading;
};

class TalonXXV;

class Vision
{
public:
    Vision(TalonXXV *pRobot);
    void LocalReset(void);
    void Analyze();
    void UpdateDash();
    void SetCameraMode(CameraMode cameraMode);

    Target GetSelectedTarget() { return m_selectedTarget; };
    double TargetFound() { return m_seesTarget; };
    
    // void SetTargetFilter(TargetFilter targetFilter) {m_targetFilter = targetFilter; };
    void SetVisionFrame(VisionFrame visionFrame) {m_visionFrame = visionFrame; };
    // void SetFiducialTargetLock(unsigned int lock_target_id){n_lockedTarget=lock_target_id;};
    // move to the target
    void DriveTargetting(double *xdot, double *ydot, double *psidot);
    void SetMiddleGoalPos(){m_goal_pos.Set(33.8, -9.5, 16.6);}; // score cubes 
    // void SetLeftGoalPos(){m_goal_pos.Set(17.0, -9.6, 190.7);};
    void SetRightGoalPos(){m_goal_pos.Set(28.6, 8.8, 16.8);}; // score cones right
    Dist2Goal GetTargetError();

private:
    TalonXXV *mainRobot;
    photonlib::PhotonCamera m_camera{"Gaelhawks_Left"};

    double m_seesTarget;
    units::time::second_t m_latency;
    double m_pipeline;

    // TargetFilter m_targetFilter;
    VisionFrame m_visionFrame;

    Target m_selectedTarget;
    unsigned int n_lockedTarget;

    Vect m_robot_pos_body, m_robot_pos_inertial, m_robot_pos_ned;
    Vect m_goal_pos;
    double m_last_pose_x;
    int stale_data_counter;
};

