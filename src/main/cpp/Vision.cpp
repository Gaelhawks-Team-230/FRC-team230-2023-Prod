#include <cmath>
#include "Common.h"
#include "Vision.h"
#include "TalonXXV.h"
#include "PoseMatrix.h"
#include "LimelightHelpers.h"
#include <chrono>
#include <units/time.h>


Vision::Vision(TalonXXV *pRobot)
{
    mainRobot = pRobot;
}

void Vision::Analyze()
{
    double l_area;
    photonlib::PhotonPipelineResult result = m_camera.GetLatestResult();
    frc::Transform3d pose;
    frc::Translation3d poseTranslation;
    frc::Rotation3d poseRotation;
    m_seesTarget = result.HasTargets();
    m_latency = result.GetLatency();

    if (m_seesTarget)
    {
        Target l_target;
        Pose l_pose;

        photonlib::PhotonTrackedTarget target = result.GetBestTarget();
        l_target.fID = target.GetFiducialId();
        l_target.ta = target.GetArea();

        frc::Transform3d pose = target.GetBestCameraToTarget();
        l_pose.x = pose.X().value() * METER_INCHES * PHOTONVISION_DIST_SF;
        l_pose.y = pose.Y().value() * METER_INCHES * PHOTONVISION_DIST_SF;
        l_pose.z = pose.Z().value() * METER_INCHES * PHOTONVISION_DIST_SF;
        // * PHOTONVISION_DIST_SF
        poseRotation = pose.Rotation();
        l_pose.rx = poseRotation.X().value() * (180.0 / M_PI);
        l_pose.ry = poseRotation.Y().value() * (180.0 / M_PI);
        l_pose.rz = poseRotation.Z().value() * (180.0 / M_PI);
        if (l_pose.x == m_last_pose_x){
            stale_data_counter++;
            printf("stale data for %f seconds\n", LOOPTIME*stale_data_counter);
        }
        else{
            stale_data_counter=0;
        }
        m_last_pose_x = l_pose.x;
        // if (mainRobot->GetLoopCount() % 100 == 0)
        // {
        //     // photon vision north, west up from the robot
        //     printf("| photonvvision | %f | %f | %f |", l_pose.x, l_pose.y, l_pose.z);
        //     printf(" %f | %f | %f |\n", l_pose.rx, l_pose.ry, l_pose.rz);
        // }
        l_target.pose = l_pose;
        m_selectedTarget = l_target;
    }
}
void Vision::UpdateDash()
{
    
    std::string l_cameraName;
    
    frc::SmartDashboard::PutNumber("Camera Latency: ", m_latency.value());
    frc::SmartDashboard::PutBoolean("Has Targets: ", m_seesTarget);
    frc::SmartDashboard::PutNumber("Ll Pipeline: ", m_pipeline);

    if (m_seesTarget)
    {
        frc::SmartDashboard::PutNumber("Fiducial ID: ", m_selectedTarget.fID);
    }
}
void Vision::LocalReset()
{
    m_seesTarget = 0.0;
    stale_data_counter = 0;
    m_latency = units::time::second_t{0};
    // m_targetFilter = BEST_TARGET;
    m_visionFrame = ROBOT_FRAME;
    n_lockedTarget = 0;
    m_pipeline = 0.0;

    m_last_pose_x=0.0;
}

void Vision::DriveTargetting(double *xdot, double *ydot, double *psidot)
{
    double vxdot, vydot, vpsidot;
    if (!m_seesTarget || m_pipeline != 0.0)
    {
        return;
    }
    Dist2Goal dist2goal;
    dist2goal = GetTargetError();
    // if (mainRobot->GetLoopCount() % 200 == 0)
    // {
    //     printf("d2g_xyz: %f, %f, %f\n", dist2goal.d2g_xyz[0], dist2goal.d2g_xyz[1], dist2goal.d2g_xyz[2]);
    //     printf("d2g_ned: %f, %f, %f\n", dist2goal.d2g_ned[0], dist2goal.d2g_ned[1], dist2goal.d2g_ned[2]);
    //     printf("psi2target: %f\n", dist2goal.psi2g_target);
    //     printf("psi_heading: %f\n", TalonXXV::Wrap(180.0 - dist2goal.heading));
    // }
    switch (m_visionFrame)
    {
    case VisionFrame::ROBOT_FRAME:
        // printf("%f, %f, %f\n", dist2goal.d2g_xyz[0], dist2goal.d2g_xyz[1], dist2goal.psi2g_target);
        vxdot = KXY * dist2goal.d2g_xyz[0];
        vydot = KXY * dist2goal.d2g_xyz[1];
        vpsidot = KPSI * TalonXXV::Wrap(180.0 - dist2goal.heading);
        break;
    case VisionFrame::FIELD_FRAME:
        vxdot = KXY * dist2goal.d2g_ned[0];
        vydot = KXY * dist2goal.d2g_ned[1];
        vpsidot = KPSI * dist2goal.psi2g_target;
        break;
    }

    *xdot += TalonXXV::Limit(-XY_VEL_LIMIT, XY_VEL_LIMIT, vxdot);
    *ydot += TalonXXV::Limit(-XY_VEL_LIMIT, XY_VEL_LIMIT, vydot);
    *psidot += TalonXXV::Limit(-R_VEL_LIMIT, R_VEL_LIMIT, vpsidot);
}
/**
 * @brief
 *
 * @return Dist2Goal
 */
Dist2Goal Vision::GetTargetError()
{
    Dist2Goal result;

    Pose p;
    p = m_selectedTarget.pose;

    Vect d2g_interial, d2g_body;
    DCM cci, cic, cib;

    Vect xyz, ptp;

    xyz.Set(p.x, p.y, p.z);
    ptp.Set(p.rx, p.ry, p.rz);

    cci = Euler2DCM(ptp[0], ptp[1], ptp[2]);
    cic = cci.Transpose();
    cib = Euler2DCM(0.0, 0.0, ptp[2] + HEADING_OFFSET);

    // m_goal_pos[1] = -m_goal_pos[1];
    // m_goal_pos[2] = -m_goal_pos[2];

    m_robot_pos_body = -xyz;
    m_robot_pos_inertial = cci * m_robot_pos_body;

    d2g_interial = m_goal_pos + (-m_robot_pos_inertial);
    m_robot_pos_ned.Set(m_robot_pos_inertial[0], -m_robot_pos_inertial[1], -m_robot_pos_inertial[2]);

    result.d2g_ned.Set(d2g_interial[0], -d2g_interial[1], -d2g_interial[2]);

    result.d2g_xyz = cib * result.d2g_ned;

    result.psi2g_target = atan2d(-xyz[1], xyz[0]);

    result.heading = ptp[2];

    // if (mainRobot->GetLoopCount() % 100 == 0)
    // {
    //     printf("robot pos:   goal pos:   d2g ned:   d2g xyz: \n");
    //     m_robot_pos_inertial.Print();
    //     // m_goal_pos.Print();
    //     // result.d2g_ned.Print();
    //     // result.d2g_xyz.Print();
    // }

    return result;
}
void Vision::SetCameraMode(CameraMode cameraMode)
{
    if (cameraMode == DRIVER_CAM)
    {
        LimelightHelpers::setPipelineIndex("limelight", 1);
    }
    else if (cameraMode == APRIL_TAG_CAM)
    {
        LimelightHelpers::setPipelineIndex("limelight", 0);
    }
}