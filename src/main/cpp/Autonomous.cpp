#include "Autonomous.h"

#include "TalonXXV.h"

Autonomous::Autonomous(TalonXXV *pRobot)
{
    mainRobot = pRobot;
    StartingConfig();
}
void Autonomous::LocalReset(void)
{
    autoStage = 0;
    countOffset = 0;
    elaspedTime = -1.0;

    StopDrive();
}
void Autonomous::StartingConfig()
{
    LocalReset();
}

/**
 * @brief Path place cone and balance via charge
 *
 */
void Autonomous::PathA()
{
    switch (autoStage)
    {
    case 0:
        StopDrive();
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        WaitForTime(0.2);
        break;
    case 1:
        mainRobot->m_grabber->GatherCone();
        WaitForTime(0.2);
        break;
    case 2:
        prepareToScore = new TrapezoidalMotion(-12.0, 120.0, 24.0);
        autoStage++;
        break;
    case 3:
        prepareToScore->Service(&xdot);
        if (prepareToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 4:
        mainRobot->m_arm->SetGoalPos(HIGH_POS);
        autoStage++;
        break;
    case 5:
        WaitForTime(1.5);
        break;
    case 6:
        moveToScore = new TrapezoidalMotion(10.0, 120.0, 24.0);
        autoStage++;
        break;
    case 7:
        moveToScore->Service(&xdot);
        if (moveToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 8:
        mainRobot->m_arm->BiasArm(-7.0);
        WaitForTime(0.2);
        break;
    case 9:
        mainRobot->m_grabber->EjectCone();
        WaitForTime(0.1);
        break;
    case 10:
        // exitCommunity = new TrapezoidalMotion(-182.0, 100.0, 60.0); // 182
        exitCommunity = new TrapezoidalMotion(-194.0, 100.0, 60.0);
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        autoStage++;
        break;
    case 11:
        exitCommunity->Service(&xdot);
        if (exitCommunity->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 12:
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        // WaitForTime(0.1);
        WaitForTime(1.0);
        break;
    case 13:
        mainRobot->m_drivetrain->PitchGyroReset();
        autoStage++;
        break;
    case 14:
        AccelProfile(1.0, 2.0);
        break;
    case 15:
        // ConstProfile(1.2, 30.0); 36
        ConstProfile(1.2, 40.0); // 48
        break;
    case 16:
        StopDrive();
        autoStage++;
        break;
    case 17:
        xdot = 0.0;
        mainRobot->m_drivetrain->AutoBalance(&xdot);
        ConstProfile(20.0, xdot);
        break;
    case 18:
        StopDrive();
        break;
    }
}

/**
 * @brief Path Place Cone
 *
 */
void Autonomous::PathB()
{
    switch (autoStage)
    {
    case 0:
        StopDrive();
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        WaitForTime(0.2);
        break;
    case 1:
        mainRobot->m_grabber->GatherCone();
        WaitForTime(0.2);
        break;
    case 2:
        prepareToScore = new TrapezoidalMotion(-12.0, 120.0, 24.0);
        autoStage++;
        break;
    case 3:
        prepareToScore->Service(&xdot);
        if (prepareToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 4:
        mainRobot->m_arm->SetGoalPos(HIGH_POS);
        autoStage++;
        break;
    case 5:
        WaitForTime(1.5);
        break;
    case 6:
        moveToScore = new TrapezoidalMotion(10.0, 120.0, 24.0);
        autoStage++;
        break;
    case 7:
        moveToScore->Service(&xdot);
        if (moveToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 8:
        mainRobot->m_arm->BiasArm(-7.0);
        WaitForTime(0.2);
        break;
    case 9:
        mainRobot->m_grabber->EjectCone();
        WaitForTime(0.1);
        break;
    case 10:
        ejectGamepiece = new TrapezoidalMotion(-12.0, 100.0, 18.0);
        autoStage++;
    case 11:
        ejectGamepiece->Service(&xdot);
        if (ejectGamepiece->CheckRoutineStatus())
        {
            StopDrive();
            autoStage++;
        }
    case 12:
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        autoStage++;
        break;
    case 13:
        StopDrive();
        autoStage++;
        break;
    case 14:
        WaitForTime(7.0);
        break;
    }
}

/**
 * @brief Path Place Cone and exit
 *
 */
void Autonomous::PathC()
{
    switch (autoStage)
    {
    case 0:
        StopDrive();
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        WaitForTime(0.2);
        break;
    case 1:
        mainRobot->m_grabber->GatherCone();
        WaitForTime(0.2);
        break;
    case 2:
        prepareToScore = new TrapezoidalMotion(-12.0, 120.0, 24.0);
        autoStage++;
        break;
    case 3:
        prepareToScore->Service(&xdot);
        if (prepareToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 4:
        mainRobot->m_arm->SetGoalPos(HIGH_POS);
        autoStage++;
        break;
    case 5:
        WaitForTime(1.5);
        break;
    case 6:
        moveToScore = new TrapezoidalMotion(10.0, 120.0, 24.0);
        autoStage++;
        break;
    case 7:
        moveToScore->Service(&xdot);
        if (moveToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 8:
        mainRobot->m_arm->BiasArm(-7.0);
        WaitForTime(0.2);
        break;
    case 9:
        mainRobot->m_grabber->EjectCone();
        WaitForTime(0.1);
        break;
    case 10:
        exitCommunity = new TrapezoidalMotion(-185.0, 100.0, 60.0);
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        autoStage++;
        break;
    case 11:
        exitCommunity->Service(&xdot);
        if (exitCommunity->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 12:
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        WaitForTime(2.0);
        break;
    case 13:
        StopDrive();
        break;
    }
}

/**
 * @brief Path Cone and Cube
 *
 */
void Autonomous::PathD()
{
    switch (autoStage)
    {
    case 0:
        StopDrive();
        if (mainRobot->m_arm->IsExtenderCalibrated())
        {
            mainRobot->m_arm->SetGoalPos(STOW_POS);
            autoStage++;
        }
        break;
    case 1:
        mainRobot->m_grabber->GatherCone();
        WaitForTime(0.2);
        break;
    case 2:
        prepareToScore = new TrapezoidalMotion(12.0, 120.0, 24.0); // 0.68 s
        mainRobot->m_arm->SetGoalPos(HIGH_POS);
        autoStage++;
        break;
    case 3:
        prepareToScore->Service(&xdot);
        if (prepareToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 4:
        autoStage++;
        break;
    case 5:
        WaitForTime(0.3);
        break;
    case 6:
        moveToScore = new TrapezoidalMotion(-10.0, 120.0, 24.0); // 0.6 seconds
        autoStage++;
        break;
    case 7:
        moveToScore->Service(&xdot);
        if (moveToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 8:
        mainRobot->m_arm->BiasArm(-7.0);
        WaitForTime(0.2);
        break;
    case 9:
        mainRobot->m_grabber->EjectCone();
        WaitForTime(0.1);
        break;
    case 10:
        ejectGamepiece = new TrapezoidalMotion(10.0, 120.0, 24.0); // 0.6 seconds
        autoStage++;
        break;
    case 11:
        ejectGamepiece->Service(&xdot);
        if (ejectGamepiece->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 12:
        mainRobot->m_arm->BiasArm(0.0);
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_grabber->GatherCube();
        mainRobot->m_arm->SetGoalPos(REARPICKUP_POS);
        autoStage++;
        break;
    case 13:
        // WaitForTime(0.75);
        mainRobot->m_grabber->GatherCube();
        autoStage++;
        StopDrive();
        break;
    // 5.08 seconds to place cone
    case 14:
        // AccelProfileDynamic(1.0, 0.75, 0.0, 0.0);
        // * Translate left/right to cube
        mainRobot->m_grabber->GatherCube();
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            TrajAccelProfile(1.0, 0.0, 0.0, 0.0, 75.0, 38.0, 0.0); // 19 // * practice field dist
            // TrajAccelProfile(1.0, 0.0, 0.0, 0.0, 75.0, 30.0, 0.0); // 15
            // TrajAccelProfile(1.0, 0.0, 0.0, 0.0, 75.0, 14.0, 0.0); //7
            // TrajAccelProfile(1.0, 0.0, 0.0, 0.0, 75.0, 20.0, 0.0); // 10 //* competition dist
        }
        else
        {
            TrajAccelProfile(1.0, 0.0, 0.0, 0.0, 75.0, -38.0, 0.0);
            // TrajAccelProfile(1.0, 0.0, 0.0, 0.0, 75.0, -30.0, 0.0);
            // TrajAccelProfile(1.0, 0.0, 0.0, 0.0, 75.0, -20.0, 0.0);
        }
        break;
    case 15:
        mainRobot->m_grabber->GatherCube();
        // TrajConstProfile(1.2, 75.0, 0.0, 0.0);  // 90
        TrajConstProfile(1.26, 75.0, 0.0, 0.0); // 94.5
        break;
    case 16:
        mainRobot->m_grabber->GatherCube();
        // mainRobot->m_arm->SetGoalPos(REARPICKUP_POS);
        autoStage++;
        break;
    case 17:
        mainRobot->m_grabber->GatherCube();
        TrajAccelProfile(1.0, 75.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
    case 18:
        mainRobot->m_grabber->GatherCube();
        StopDrive();
        WaitForTime(0.2);
        break;
    case 19:
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        autoStage++;
        break;
    case 20:
        TrajAccelProfile(1.0, 0.0, 0.0, 0.0, -75.0, 0.0, 0.0);
        break;
    case 21:
        // TrajConstProfile(1.27, -75.0, 0.0, 0.0); // 95.25
        TrajConstProfile(1.33, -75.0, 0.0, 0.0); // 99.75
        break;
    case 22:
        mainRobot->m_arm->SetGoalPos(HIGH_POS);
        autoStage++;
        break;
    case 23:
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            // TrajAccelProfile(1.0, -75.0, 6.0, 0.0, 0.0, 0.0, 0.0);
            TrajAccelProfile(1.0, -75.0, 10.0, 0.0, 0.0, 0.0, 0.0);
        }
        else
        {
            // TrajAccelProfile(1.0, -75.0, -6.0, 0.0, 0.0, 0.0, 0.0);
            TrajAccelProfile(1.0, -75.0, -10.0, 0.0, 0.0, 0.0, 0.0);
        }
        break;
    case 24:
        StopDrive();
        WaitForTime(0.7);
        break;
    case 25:
        mainRobot->m_grabber->EjectCube();
        WaitForTime(0.4);
        break;
    case 26:
        ejectGamepiece = new TrapezoidalMotion(10.0, 120.0, 24.0); // 0.6 seconds
        StopDrive();
        autoStage++;
        break;
    case 27:
        ejectGamepiece->Service(&xdot);
        if (ejectGamepiece->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 28:
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        autoStage++;
        break;
    case 29:
        WaitForTime(2.0);
        break;
    case 30:
        StopDrive();
        break;
    }
}

/**
 * @brief Path cone, cube and balance via charge
 *
 */
void Autonomous::PathE()
{
    switch (autoStage)
    {
    case 0:
        StopDrive();
        if (mainRobot->m_arm->IsExtenderCalibrated())
        {
            mainRobot->m_arm->SetGoalPos(STOW_POS);
            autoStage++;
        }
        break;
    case 1:
        mainRobot->m_grabber->GatherCone();
        WaitForTime(0.2);
        break;
    case 2:
        prepareToScore = new TrapezoidalMotion(12.0, 120.0, 24.0); // 0.68 s
        mainRobot->m_arm->SetGoalPos(HIGH_POS);
        autoStage++;
        break;
    case 3:
        prepareToScore->Service(&xdot);
        if (prepareToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 4:
        autoStage++;
        break;
    case 5:
        WaitForTime(0.3);
        break;
    case 6:
        moveToScore = new TrapezoidalMotion(-10.0, 120.0, 24.0); // 0.6 seconds
        autoStage++;
        break;
    case 7:
        moveToScore->Service(&xdot);
        if (moveToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 8:
        mainRobot->m_arm->BiasArm(-7.0);
        WaitForTime(0.2);
        break;
    case 9:
        mainRobot->m_grabber->EjectCone();
        WaitForTime(0.1);
        break;
    case 10:
        ejectGamepiece = new TrapezoidalMotion(10.0, 120.0, 24.0); // 0.6 seconds
        autoStage++;
        break;
    case 11:
        ejectGamepiece->Service(&xdot);
        if (ejectGamepiece->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 12:
        mainRobot->m_arm->BiasArm(0.0);
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_grabber->GatherCube();
        mainRobot->m_arm->SetGoalPos(REARPICKUP_POS);
        autoStage++;
        break;
    case 13:
        mainRobot->m_grabber->GatherCube();
        autoStage++;
        StopDrive();
        break;
    // 5.08 seconds to place cone
    case 14:
        mainRobot->m_grabber->GatherCube();
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            // TrajAccelProfile(0.75, 0.0, 0.0, 0.0, 100.0, 50.6, 0.0); // 18.9 //* Competition distance
            // TrajAccelProfile(0.75, 0.0, 0.0, 0.0, 100.0, 39.9, 0.0); // 14.9
            // TrajAccelProfile(0.75, 0.0, 0.0, 0.0, 100.0, 18.7, 0.0); // 7.0
            TrajAccelProfile(0.75, 0.0, 0.0, 0.0, 100.0, 26.6, 0.0); // * 9.975 Boston Distance
        }
        else
        {
            // TrajAccelProfile(0.75, 0.0, 0.0, 0.0, 100.0, -50.6, 0.0); // * Competition distance
            // TrajAccelProfile(0.75, 0.0, 0.0, 0.0, 100.0, -39.9, 0.0);
            TrajAccelProfile(0.75, 0.0, 0.0, 0.0, 100.0, -26.6, 0.0); // * 9.975 Boston distance
        }
        break;
    case 15:
        mainRobot->m_grabber->GatherCube();
        // TrajConstProfile(0.9, 100.0, 0.0, 0.0);  // 90
        TrajConstProfile(0.945, 100.0, 0.0, 0.0); // 94.5
        break;
    case 16:
        mainRobot->m_grabber->GatherCube();
        autoStage++;
        break;
    case 17:
        mainRobot->m_grabber->GatherCube();
        TrajAccelProfile(0.75, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
    case 18:
        mainRobot->m_grabber->GatherCube();
        StopDrive();
        WaitForTime(0.2);
        break;
    case 19:
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_arm->SetGoalPos(HIGH_POS);
        autoStage++;
        break;
    case 20:
        TrajAccelProfile(0.75, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0);
        break;
    case 21:
        // TrajConstProfile(0.68, -100.0, 0.0, 0.0); // 68
        TrajConstProfile(0.725, -100.0, 0.0, 0.0); // 72.5
        break;
    case 22:
        // mainRobot->m_arm->SetGoalPos(HIGH_POS);
        autoStage++;
        break;
    case 23:
        TrajConstProfile(0.28, -100.0, 0.0, 0.0);
        break;
    case 24:
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            // ! Not sure if should change
            // TrajAccelProfile(0.75, -100.0, 8.0, 0.0, 0.0, 0.0, 0.0);
            TrajAccelProfile(0.75, -100.0, 13.4, 0.0, 0.0, 0.0, 0.0);
        }
        else
        {
            // TrajAccelProfile(0.75, -100.0, -8.0, 0.0, 0.0, 0.0, 0.0);
            TrajAccelProfile(0.75, -100.0, -13.4, 0.0, 0.0, 0.0, 0.0);
        }
        break;
    case 25:
        StopDrive();
        WaitForTime(0.4);
        break;
    case 26:
        mainRobot->m_drivetrain->PitchGyroReset();
        autoStage++;
        break;
    case 27:
        mainRobot->m_grabber->EjectCube();
        WaitForTime(0.4);
        break;
    case 28:
        ejectGamepiece = new TrapezoidalMotion(10.0, 120.0, 24.0); // 0.6 seconds
        StopDrive();
        autoStage++;
        break;
    case 29:
        ejectGamepiece->Service(&xdot);
        if (ejectGamepiece->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 30:
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        autoStage++;
        break;
    case 31:
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            TrajConstProfile(1.0, 0.0, 60.5, 0.0);
        }
        else
        {
            TrajConstProfile(1.0, 0.0, -60.5, 0.0);
        }
        break;
    case 32:
        StopDrive();
        autoStage++;
        break;
    case 33:
        // 99 inches from grid to center charge and 89 from last robot position
        // traj plan going 122.5
        TrajAccelProfile(0.5, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0);
        break;
    case 34:
        TrajConstProfile(0.64, 100.0, 0.0, 0.0);
        break;
    case 35:
        TrajAccelProfile(0.5, 100.0, 0.0, 0.0, 40.0, 0.0, 0.0);
        break;
    case 36:
        StopDrive();
        autoStage++;
        break;
    case 37:
        xdot = 0.0;
        mainRobot->m_drivetrain->AutoBalance(&xdot);
        ConstProfile(20.0, -xdot); // * Note negate xdot as in field coordinates - HACK
        // printf("%f\n", (mainRobot->GetLoopCount()*LOOPTIME));
        if ((mainRobot->GetLoopCount()*LOOPTIME) > 14.68)
        {
            autoStage++;
            elaspedTime = -1.0;
            printf("Locking wheels\n");
        }
        break;
    case 38:
        StopDrive();
        mainRobot->m_drivetrain->DisableGyro();
        autoStage++;
        break;
    case 39:
        psidot = 50.0;
        // printf("Locking - %f\n", (mainRobot->GetLoopCount()*LOOPTIME));
        break;
    }
}

/**
 * @brief Path cone, cube, and cube
 *
 */
void Autonomous::PathF()
{
    switch (autoStage)
    {
    case 0:
        StopDrive();
        if (mainRobot->m_arm->IsExtenderCalibrated())
        {
            mainRobot->m_arm->SetGoalPos(STOW_POS);
            autoStage++;
        }
        break;
    case 1:
        mainRobot->m_grabber->GatherCone();
        WaitForTime(0.2);
        break;
    case 2:
        // move back for arm clearance
        prepareToScore = new TrapezoidalMotion(12.0, 120.0, 24.0); // 0.68 s
        mainRobot->m_arm->SetGoalPos(HIGH_POS);
        autoStage++;
        break;
    case 3:
        prepareToScore->Service(&xdot);
        if (prepareToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 4:
        WaitForTime(0.3);
        break;
    case 5:
        // move forward to cone node
        moveToScore = new TrapezoidalMotion(-10.0, 120.0, 24.0); // 0.6 seconds
        autoStage++;
        break;
    case 6:
        moveToScore->Service(&xdot);
        if (moveToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 7:
        mainRobot->m_arm->BiasArm(-7.0);
        WaitForTime(0.2);
        break;
    case 8:
        mainRobot->m_grabber->EjectCone();
        WaitForTime(0.1);
        break;
    case 9:
        // back up to eject cone
        ejectGamepiece = new TrapezoidalMotion(10.0, 120.0, 24.0); // 0.6 seconds
        autoStage++;
        break;
    case 10:
        ejectGamepiece->Service(&xdot);
        if (ejectGamepiece->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 11:
        mainRobot->m_arm->BiasArm(0.0);
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_grabber->GatherCube();
        mainRobot->m_arm->SetGoalPos(REARPICKUP_POS);
        autoStage++;
        break;
    case 12:
        mainRobot->m_grabber->GatherCube();
        autoStage++;
        StopDrive();
        break;
    // 5.08 seconds to place cone
    case 13:
        mainRobot->m_grabber->GatherCube();
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            // TrajAccelProfile(0.74, 0.0, 0.0, 0.0, 100.0, 50.6, 0.0); // 18.9 // * Competition distance
            // TrajAccelProfile(0.74, 0.0, 0.0, 0.0, 100.0, 39.9, 0.0); // 14.9
            // TrajAccelProfile(0.74, 0.0, 0.0, 0.0, 100.0, 18.7, 0.0); // 7.0
            TrajAccelProfile(0.74, 0.0, 0.0, 0.0, 100.0, 26.6, 0.0); // *  9.824 Boston distance
        }
        else
        {
            // TrajAccelProfile(0.74, 0.0, 0.0, 0.0, 100.0, -50.6, 0.0); // * practice field dist
            // TrajAccelProfile(0.74, 0.0, 0.0, 0.0, 100.0, -39.9, 0.0);
            TrajAccelProfile(0.74, 0.0, 0.0, 0.0, 100.0, -26.6, 0.0); // * Boston distance
        }
        break;
    case 14:
        mainRobot->m_grabber->GatherCube();
        // constant vel to gamepiece 94.5 in.
        // added 8 inches (0.08 to time) for going to cube
        TrajConstProfile(1.02, 100.0, 0.0, 0.0); // 94.5
        break;
    case 15:
        mainRobot->m_grabber->GatherCube();
        // decelerate to 0 to gamepiece for 37.5 in.
        TrajAccelProfile(0.74, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
    case 16:
        mainRobot->m_grabber->GatherCube();
        StopDrive();
        WaitForTime(0.2);
        break;
    case 17:
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_arm->SetGoalPos(HIGH_POS);
        autoStage++;
        break;
    case 18:
        // accelerate in neg direction back to node for 37.5 in.
        TrajAccelProfile(0.74, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0);
        break;
    case 19:
        // constant vel back to node for 72.5 in.
        // added 8 inches for going back to node
        TrajConstProfile(0.8, -100.0, 0.0, 0.0); // 72.5
        break;
    case 20:
        // decelerate to 0 back to node for 28 in.
        TrajConstProfile(0.28, -100.0, 0.0, 0.0);
        break;
    case 21:
        // charge station clearance 37.5 in. x and +/- 3 (5 NOW) in. y
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            // TrajAccelProfile(0.74, -100.0, 8.0, 0.0, 0.0, 0.0, 0.0);
            TrajAccelProfile(0.74, -100.0, 13.6, 0.0, 0.0, 0.0, 0.0);
        }
        else
        {
            // TrajAccelProfile(0.74, -100.0, -8.0, 0.0, 0.0, 0.0, 0.0);
            TrajAccelProfile(0.74, -100.0, -13.6, 0.0, 0.0, 0.0, 0.0);
        }
        break;
    case 22:
        StopDrive();
        WaitForTime(0.4);
        break;
    case 23:
        mainRobot->m_grabber->EjectCube();
        WaitForTime(0.4);
        break;
    // * start third gamepiece
    case 24:
        StopDrive();
        autoStage++;
        break;
    case 25:
        // move back for eject
        ejectGamepiece = new TrapezoidalMotion(10.0, 120.0, 24.0); // 0.6 seconds
        autoStage++;
        break;
    case 26:
        ejectGamepiece->Service(&xdot);
        if (ejectGamepiece->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 27:
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_grabber->GatherCone();
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        autoStage++;
        break;
    case 28:
        mainRobot->m_grabber->GatherCone();
        // accelerate back and clear charge station, 37.5 in. x and +/- 3 (5 NOW) in. y
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            // TrajAccelProfile(0.74, 0.0, 0.0, 0.0, 100.0, 8.0, 0.0);
            TrajAccelProfile(0.74, 0.0, 0.0, 0.0, 100.0, -13.6, 0.0);
        }
        else
        {
            // TrajAccelProfile(0.74, 0.0, 0.0, 0.0, 100.0, -8.0, 0.0);
            TrajAccelProfile(0.74, 0.0, 0.0, 0.0, 100.0, 13.6, 0.0);
            // * SWAPPED SIGNS
        }
        break;
    case 29:
        mainRobot->m_grabber->GatherCone();
        mainRobot->m_arm->SetGoalPos(PICKUP_POS);
        // constant vel back for 94.5 in.
        TrajConstProfile(1.24, 100.0, 0.0, 0.0);
        break;
    case 30:
        mainRobot->m_grabber->GatherCone();
        // circle around the charge station to gather cube
        // x = 37.5 inches
        // y = 48 inches
        // psi = 45 deg
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            // TrajAccelProfile(0.75, 100.0, 128.0, 120.0, 0.0, 0.0, 0.0);
            // TrajAccelProfile(1.5, 75.0, 96.0, 90.0, 0.0, 0.0, 0.0);
            // TrajAccelProfile(2.0, 37.5, 48.0, 45.0, 0.0, 0.0, 0.0);
            // TrajAccelProfile(2.0, 37.5, 36.0, 45.0, 0.0, 0.0, 0.0);
            TrajAccelProfile(1.0, 75.0, 24.0, -180.0, 0.0, 24.0, 0.0);
        }
        else
        {
            // TrajAccelProfile(0.75, 100.0, -128.0, -120.0, 0.0, 0.0, 0.0);
            // TrajAccelProfile(1.5, 75.0, -96.0, -90.0, 0.0, 0.0, 0.0);
            // TrajAccelProfile(2.0, 37.5, -48.0, -45.0, 0.0, 0.0, 0.0);
            TrajAccelProfile(1.0, 75.0, -24.0, 180.0, 0.0, -24.0, 0.0);
        }
        break;
    case 31:
    {
        // go -12 inches y
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            TrajConstProfile(0.5, 0.0, 34.0, 0.0);
        }
        else
        {
            TrajConstProfile(0.5, 0.0, -34.0, 0.0);
        }
        break;
    }
    case 32:
        WaitForTime(0.3);
        StopDrive();
        break;
    case 33:
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        autoStage++;
        break;
    case 34:
        WaitForTime(2.0);
        break;
    case 35:
        StopDrive();
        break;

        // case 31:
        //     mainRobot->m_grabber->GatherCone();
        //     StopDrive();
        //     WaitForTime(0.2);
        //     break;
        // case 32:
        //     mainRobot->m_grabber->SetHoldState(false);
        //     mainRobot->m_grabber->StopRollers();
        //     mainRobot->m_arm->SetGoalPos(MID_POS);
        //     autoStage++;
        //     break;
        // case 33:
        //     // undo circle to get back in straight path towards cube node
        //     if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        //     {
        //         // TrajAccelProfile(0.75, 0.0, 0.0, 0.0, -100.0, -128.0, -120.0);
        //         // TrajAccelProfile(1.5, 0.0, 0.0, 0.0, -75.0, -96.0, -90.0);
        //         // TrajAccelProfile(2.0, 0.0, 0.0, 0.0, -37.5, -48.0, -45.0);
        //         // TrajAccelProfile(2.0, 0.0, 0.0, 0.0, -37.5, -36.0, -45.0);
        //         TrajAccelProfile(2.0, 0.0, 0.0, 0.0, -37.5, 0.0, 90.0);
        //     }
        //     else
        //     {
        //         // TrajAccelProfile(0.75, 0.0, 0.0, 0.0, -100.0, 128.0, 120.0);
        //         // TrajAccelProfile(1.5, 0.0, 0.0, 0.0, -75.0, 96.0, 90.0);
        //         // TrajAccelProfile(2.0, 0.0, 0.0, 0.0, -37.5, 48.0, 45.0);
        //         // TrajAccelProfile(2.0, 0.0, 0.0, 0.0, -37.5, 36.0, 45.0);
        //         TrajAccelProfile(2.0, 0.0, 0.0, 0.0, -37.5, 36.0, -90.0);
        //     }
        //     break;
        // case 34:
        //     // constant vel back to node for 72.5 in.
        //     TrajConstProfile(0.725, -100.0, 0.0, 0.0);
        //     break;
        // case 35:
        //     // decelerate back to node for 28 in.
        //     TrajConstProfile(0.28, -100.0, 0.0, 0.0);
        //     break;
        // case 36:
        //     // charge station clearance 37.5 in. x and +/- 3 in. y
        //     if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        //     {
        //         TrajAccelProfile(0.75, -100.0, 8.0, 0.0, 0.0, 0.0, 0.0);
        //     }
        //     else
        //     {
        //         TrajAccelProfile(0.75, -100.0, -8.0, 0.0, 0.0, 0.0, 0.0);
        //     }
        //     break;
        // case 37:
        //     StopDrive();
        //     WaitForTime(0.4);
        //     break;
        // case 38:
        //     mainRobot->m_grabber->EjectCone();
        //     WaitForTime(0.4);
        //     break;
        // case 39:
        //     // move back for ejecting
        //     ejectGamepiece = new TrapezoidalMotion(10.0, 120.0, 24.0); // 0.6 seconds
        //     StopDrive();
        //     autoStage++;
        //     break;
        // case 40:
        //     ejectGamepiece->Service(&xdot);
        //     if (ejectGamepiece->CheckRoutineStatus())
        //     {
        //         autoStage++;
        //         StopDrive();
        //     }
        //     break;
        // case 41:
        //     mainRobot->m_grabber->SetHoldState(false);
        //     mainRobot->m_grabber->StopRollers();
        //     mainRobot->m_arm->SetGoalPos(STOW_POS);
        //     autoStage++;
        //     break;
        // case 42:
        //     WaitForTime(2.0);
        //     break;
        // case 43:
        //     StopDrive();
        //     break;
    }
}

/**
 * @brief Path cone and cube dirty side | DEPRICATED (Do not use)
 */
void Autonomous::PathG()
{
    switch (autoStage)
    {
    case 0:
        StopDrive();
        if (mainRobot->m_arm->IsExtenderCalibrated())
        {
            mainRobot->m_arm->SetGoalPos(STOW_POS);
            autoStage++;
        }
        break;
    case 1:
        mainRobot->m_grabber->GatherCone();
        WaitForTime(0.2);
        break;
    case 2:
        prepareToScore = new TrapezoidalMotion(12.0, 120.0, 24.0); // 0.68 s
        mainRobot->m_arm->SetGoalPos(HIGH_POS);
        autoStage++;
        break;
    case 3:
        prepareToScore->Service(&xdot);
        if (prepareToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 4:
        WaitForTime(0.3);
        break;
    case 5:
        moveToScore = new TrapezoidalMotion(-10.0, 120.0, 24.0); // 0.6 seconds
        autoStage++;
        break;
    case 6:
        moveToScore->Service(&xdot);
        if (moveToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 7:
        mainRobot->m_arm->BiasArm(-7.0);
        WaitForTime(0.2);
        break;
    case 8:
        mainRobot->m_grabber->EjectCone();
        WaitForTime(0.1);
        break;
    case 9:
        ejectGamepiece = new TrapezoidalMotion(10.0, 120.0, 24.0); // 0.6 seconds
        autoStage++;
        break;
    case 10:
        ejectGamepiece->Service(&xdot);
        if (ejectGamepiece->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    // * 3.0 seconds
    case 11:
        mainRobot->m_arm->BiasArm(0.0);
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        // mainRobot->m_grabber->GatherCube();
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        StopDrive();
        autoStage++;
        break;
    case 12:
        // * Translate left/right to cube
        // mainRobot->m_grabber->GatherCube();
        // * 37.5 inches foreward
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            // TrajAccelProfile(1.0, 0.0, 0.0, 0.0, 75.0, -38.0, 0.0); // 19 inches horz
            TrajAccelProfile(1.0, 0.0, 0.0, 0.0, 75.0, -24.0, 0.0); // 12 inches horz
        }
        else
        {
            TrajAccelProfile(1.0, 0.0, 0.0, 0.0, 75.0, 24.0, 0.0);
        }
        break;
    case 13:
        // * 12.5 inches foreward
        TrajConstProfile(0.16, 75.0, 0.0, 0.0);
        break;
    case 14:
        // * 27.5 inches
        TrajAccelProfile(0.5, 75.0, 0.0, 0.0, 35.0, 0.0, 0.0);
        break;
    case 15:
        // * 20 inches where middle clears bump
        // ! bump cleared
        TrajConstProfile(0.58, 35.0, 0.0, 0.0);
        break;
    case 16:
        mainRobot->m_arm->SetGoalPos(REARPICKUP_POS);
        autoStage++;
        break;
    case 17:
        // * 27.5 inches
        TrajAccelProfile(0.5, 35.0, 0.0, 0.0, 55.0, 0.0, 0.0);
        mainRobot->m_grabber->GatherCube();
        break;
    case 18:
        // * 59.5 inches
        TrajConstProfile(1.36, 55.0, 0.0, 0.0);
        mainRobot->m_grabber->GatherCube();
        break;
    case 19:
        // * 37.5 inches
        mainRobot->m_grabber->GatherCube();
        TrajAccelProfile(1.0, 75.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
    case 20:
        mainRobot->m_grabber->GatherCube();
        StopDrive();
        WaitForTime(0.2);
        break;
    case 21:
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        autoStage++;
        break;
    // * Halfway point
    case 22:
        // * -37.5 inches
        TrajAccelProfile(1.0, 0.0, 0.0, 0.0, -75.0, 0.0, 0.0);
        break;
    case 23:
        // * -59.5 inches
        TrajConstProfile(0.8, -75.0, 0.0, 0.0);
        break;
    case 24:
        // * -27.5 inches
        TrajAccelProfile(0.5, -75.0, 0.0, 0.0, -35.0, 0.0, 0.0);
        break;
    case 25:
        // * -20 inches where middle clears bump
        // ! bump cleared
        TrajConstProfile(0.58, -35.0, 0.0, 0.0);
        break;
    case 26:
        mainRobot->m_arm->SetGoalPos(HIGH_POS);
        autoStage++;
        break;
    case 27:
        // * -27.5 inches
        TrajAccelProfile(0.5, -35.0, 0.0, 0.0, -75.0, 0.0, 0.0);
        break;
    case 28:
        // * -12.5 inches foreward
        TrajConstProfile(0.16, -75.0, 0.0, 0.0);
        break;
    case 29:
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            TrajAccelProfile(1.0, -75.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
        else
        {
            TrajAccelProfile(1.0, -75.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
        break;
    case 30:
        StopDrive();
        WaitForTime(0.7);
        break;
    case 31:
        mainRobot->m_grabber->EjectCube();
        WaitForTime(0.4);
        break;
    case 32:
        ejectGamepiece = new TrapezoidalMotion(10.0, 120.0, 24.0); // 0.6 seconds
        autoStage++;
        break;
    case 33:
        ejectGamepiece->Service(&xdot);
        if (ejectGamepiece->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 34:
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        autoStage++;
        break;
    case 35:
        WaitForTime(2.0);
        break;
    case 36:
        StopDrive();
        break;
    }
}

/**
 * @brief Path cone, cube, and cube | Cable protector side
 */
void Autonomous::PathH()
{
    switch (autoStage)
    {
    case 0:
        StopDrive();
        if (mainRobot->m_arm->IsExtenderCalibrated())
        {
            mainRobot->m_arm->SetGoalPos(STOW_POS);
            autoStage++;
        }
        break;
    case 1:
        mainRobot->m_grabber->GatherCone();
        WaitForTime(0.2);
        break;
    case 2:
        // move back for arm clearance
        prepareToScore = new TrapezoidalMotion(12.0, 120.0, 24.0); // 0.68 s
        mainRobot->m_arm->SetGoalPos(HIGH_POS);
        autoStage++;
        break;
    case 3:
        prepareToScore->Service(&xdot);
        if (prepareToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 4:
        WaitForTime(0.3);
        break;
    case 5:
        // move forward to cone node
        moveToScore = new TrapezoidalMotion(-10.0, 120.0, 24.0); // 0.6 seconds
        autoStage++;
        break;
    case 6:
        moveToScore->Service(&xdot);
        if (moveToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 7:
        mainRobot->m_arm->BiasArm(-7.0);
        WaitForTime(0.2);
        break;
    case 8:
        mainRobot->m_grabber->EjectCone();
        WaitForTime(0.1);
        break;
    case 9:
        // back up to eject cone
        ejectGamepiece = new TrapezoidalMotion(10.0, 120.0, 24.0); // 0.6 seconds
        autoStage++;
        break;
    case 10:
        ejectGamepiece->Service(&xdot);
        if (ejectGamepiece->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 11:
        mainRobot->m_arm->BiasArm(0.0);
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_grabber->GatherCube();
        mainRobot->m_arm->SetGoalPos(REARPICKUP_POS);
        autoStage++;
        break;
    case 12:
        mainRobot->m_grabber->GatherCube();
        autoStage++;
        StopDrive();
        break;
    // 3 secs to place cone
    case 13:
        mainRobot->m_grabber->GatherCube();
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            TrajAccelProfile(0.74 * PATH_H_TIME_S, 0.0, 0.0, 0.0, 100.0 * PATH_H_SPEED_SF, -(PATH_H_SPEED_SF * HORZ_VEL_TO_CUBE), 0.0);
        }
        else
        {
            TrajAccelProfile(0.74 * PATH_H_TIME_S, 0.0, 0.0, 0.0, 100.0 * PATH_H_SPEED_SF, (PATH_H_SPEED_SF * HORZ_VEL_TO_CUBE), 0.0);
        }
        break;
    case 14:
        mainRobot->m_grabber->GatherCube();
        // constant vel to gamepiece 94.5 in.
        // added 8 inches (0.08 to time) for going to cube
        TrajConstProfile(1.20 * PATH_H_TIME_S, 100.0 * PATH_H_SPEED_SF, 0.0, 0.0); // 94.5
        break;
    case 15:
        mainRobot->m_grabber->GatherCube();
        // decelerate to 0 to gamepiece for 37.5 in.
        TrajAccelProfile(0.74 * PATH_H_TIME_S, 100.0 * PATH_H_SPEED_SF, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
    case 16:
        mainRobot->m_grabber->GatherCube();
        StopDrive();
        // ! Potential to remove this 
        // WaitForTime(0.2);
        autoStage++;
        break;
    case 17:
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_arm->SetGoalPos(HIGH_POS);
        autoStage++;
        break;
    case 18:
        // accelerate in neg direction back to node for 37.5 in.
        TrajAccelProfile(0.74 * PATH_H_TIME_S_SLOW, 0.0, 0.0, 0.0, -100.0 * PATH_H_SPEED_SF_SLOW, 0.0, 0.0);
        break;
    case 19:
        // constant vel back to node for 72.5 in.
        // added 8 inches for going back to node
        TrajConstProfile(1.0 * PATH_H_TIME_S_SLOW, -100.0 * PATH_H_SPEED_SF_SLOW, 0.0, 0.0); // 72.5
        break;
    case 20:
        // decelerate to 0 back to node for 28 in.
        TrajConstProfile(0.28 * PATH_H_TIME_S_SLOW, -100.0 * PATH_H_SPEED_SF_SLOW, 0.0, 0.0);
        break;
    case 21:
        // charge station clearance 37.5 in. x and +/- 3 (5 NOW) in. y
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            TrajAccelProfile(0.74 * PATH_H_TIME_S_SLOW, -100.0 * PATH_H_SPEED_SF_SLOW, -13.6 * PATH_H_SPEED_SF_SLOW, 0.0, 0.0, 0.0, 0.0);
        }
        else
        {
            TrajAccelProfile(0.74 * PATH_H_TIME_S_SLOW, -100.0 * PATH_H_SPEED_SF_SLOW, 13.6 * PATH_H_SPEED_SF_SLOW, 0.0, 0.0, 0.0, 0.0);
        }
        break;
    case 22:
        StopDrive();
        // ! Potential to not need as much time for robots momentum to stop
        WaitForTime(0.2);
        break;
    case 23:
        mainRobot->m_grabber->EjectCube();
        WaitForTime(0.4);
        break;
    // * start third gamepiece
    case 24:
        StopDrive();
        autoStage++;
        break;
    case 25:
        // move back for eject
        ejectGamepiece = new TrapezoidalMotion(10.0, 120.0, 24.0); // 0.6 seconds
        autoStage++;
        break;
    case 26:
        ejectGamepiece->Service(&xdot);
        if (ejectGamepiece->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 27:
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_grabber->GatherCone();
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        autoStage++;
        break;
    case 28:
        mainRobot->m_grabber->GatherCone();
        // accelerate back and clear charge station, 37.5 in. x and +/- 3 (5 NOW) in. y
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            TrajAccelProfile(0.74 * PATH_H_TIME_S, 0.0, 0.0, 0.0, 100.0 * PATH_H_SPEED_SF, 13.6 * PATH_H_SPEED_SF, 0.0);
        }
        else
        {
            TrajAccelProfile(0.74 * PATH_H_TIME_S, 0.0, 0.0, 0.0, 100.0 * PATH_H_SPEED_SF, -13.6 * PATH_H_SPEED_SF, 0.0);
        }
        break;
    case 29:
        mainRobot->m_grabber->GatherCone();
        mainRobot->m_arm->SetGoalPos(PICKUP_POS);
        // constant vel back for 94.5 in.
        TrajConstProfile(1.24 * PATH_H_TIME_S, 100.0 * PATH_H_SPEED_SF, 0.0, 0.0);
        break;
    case 30:
        mainRobot->m_grabber->GatherCone();
        // circle around the charge station to gather cube
        // x = 37.5 inches
        // y = 48 inches
        // psi = 45 deg
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            TrajAccelProfile(1.0, 75.0, -24.0, 180.0, 0.0, -24.0, 0.0);
        }
        else
        {
            TrajAccelProfile(1.0, 75.0, 24.0, -180.0, 0.0, 24.0, 0.0);
        }
        break;
    case 31:
    {
        // go -12 inches y
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            TrajConstProfile(0.5, 0.0, -34.0, 0.0);
        }
        else
        {
            TrajConstProfile(0.5, 0.0, 34.0, 0.0);
        }
        break;
    }
    case 32:
        WaitForTime(0.3);
        StopDrive();
        break;
    case 33:
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        autoStage++;
        break;
    case 34:
        WaitForTime(2.0);
        break;
    case 35:
        StopDrive();
        break;
    }
}

/**
 * @brief Path nosemode cone, cube and balance | DEPRICATED (Do not use)
 *
 */
void Autonomous::PathI()
{
    switch (autoStage)
    {
    case 0:
        StopDrive();
        mainRobot->m_arm->SetNoseMode();
        if (mainRobot->m_arm->IsExtenderCalibrated())
        {
            mainRobot->m_arm->SetGoalPos(STOW_POS);
            autoStage++;
        }
        break;
    case 1:
        mainRobot->m_grabber->GatherCone();
        WaitForTime(0.2);
        break;
    case 2:
        prepareToScore = new TrapezoidalMotion(12.0, 120.0, 24.0);
        mainRobot->m_arm->SetGoalPos(HIGH_POS);
        autoStage++;
        break;
    case 3:
        prepareToScore->Service(&xdot);
        if (prepareToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 4:
        autoStage++;
        break;
    case 5:
        WaitForTime(0.3);
        break;
    case 6:
        moveToScore = new TrapezoidalMotion(-10.0, 120.0, 24.0);
        autoStage++;
        break;
    case 7:
        moveToScore->Service(&xdot);
        if (moveToScore->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 8:
        mainRobot->m_grabber->EjectConeFaster();
        WaitForTime(0.1);
        break;
    case 9:
        mainRobot->m_arm->ClearNoseMode();
        ejectGamepiece = new TrapezoidalMotion(10.0, 120.0, 24.0); 
        autoStage++;
        break;
    case 10:
        ejectGamepiece->Service(&xdot);
        if (ejectGamepiece->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 11:
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_grabber->GatherCube();
        mainRobot->m_arm->SetGoalPos(REARPICKUP_POS);
        autoStage++;
        break;
    case 12:
        mainRobot->m_grabber->GatherCube();
        autoStage++;
        StopDrive();
        break;
    case 13:
        mainRobot->m_grabber->GatherCube();
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            TrajAccelProfile(0.75, 0.0, 0.0, 0.0, 100.0, 50.6, 0.0); 
        }
        else
        {
            TrajAccelProfile(0.75, 0.0, 0.0, 0.0, 100.0, -50.6, 0.0); 
        }
        break;
    case 14:
        mainRobot->m_grabber->GatherCube();
        TrajConstProfile(0.945, 100.0, 0.0, 0.0); 
        break;
    case 15:
        mainRobot->m_grabber->GatherCube();
        autoStage++;
        break;
    case 16:
        mainRobot->m_grabber->GatherCube();
        TrajAccelProfile(0.75, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        break;
    case 17:
        mainRobot->m_grabber->GatherCube();
        StopDrive();
        WaitForTime(0.2);
        break;
    case 18:
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_arm->SetGoalPos(HIGH_POS);
        autoStage++;
        break;
    case 19:
        TrajAccelProfile(0.75, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0);
        break;
    case 20:
        TrajConstProfile(0.725, -100.0, 0.0, 0.0); 
        break;
    case 21:
        autoStage++;
        break;
    case 22:
        TrajConstProfile(0.28, -100.0, 0.0, 0.0);
        break;
    case 23:
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            TrajAccelProfile(0.75, -100.0, 13.4, 0.0, 0.0, 0.0, 0.0);
        }
        else
        {
            TrajAccelProfile(0.75, -100.0, -13.4, 0.0, 0.0, 0.0, 0.0);
        }
        break;
    case 24:
        StopDrive();
        WaitForTime(0.4);
        break;
    case 25:
        mainRobot->m_drivetrain->PitchGyroReset();
        autoStage++;
        break;
    case 26:
        mainRobot->m_grabber->EjectCube();
        WaitForTime(0.4);
        break;
    case 27:
        ejectGamepiece = new TrapezoidalMotion(10.0, 120.0, 24.0); 
        StopDrive();
        autoStage++;
        break;
    case 28:
        ejectGamepiece->Service(&xdot);
        if (ejectGamepiece->CheckRoutineStatus())
        {
            autoStage++;
            StopDrive();
        }
        break;
    case 29:
        mainRobot->m_grabber->SetHoldState(false);
        mainRobot->m_grabber->StopRollers();
        mainRobot->m_arm->SetGoalPos(STOW_POS);
        autoStage++;
        break;
    case 30:
        if (mainRobot->GetAlliance() == frc::DriverStation::Alliance::kBlue)
        {
            TrajConstProfile(1.0, 0.0, 60.5, 0.0);
        }
        else
        {
            TrajConstProfile(1.0, 0.0, -60.5, 0.0);
        }
        break;
    case 31:
        StopDrive();
        autoStage++;
        break;
    case 32:
        TrajAccelProfile(0.5, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0);
        break;
    case 33:
        TrajConstProfile(0.64, 100.0, 0.0, 0.0);
        break;
    case 34:
        TrajAccelProfile(0.5, 100.0, 0.0, 0.0, 40.0, 0.0, 0.0);
        break;
    case 35:
        StopDrive();
        break;
    }
}

/**
 * @brief Wait for a specified amount of time
 * 97.5 inches from grid to bump 87.5
 * 224 inches from grid to gamepiece - 126.5 inches past bump to game piece
 *
 * @param time Time to wait in seconds
 */
void Autonomous::WaitForTime(double time)
{
    if (elaspedTime == -1.0)
    {
        countOffset = mainRobot->GetLoopCount();
    }
    elaspedTime = (mainRobot->GetLoopCount() - countOffset) * LOOPTIME;
    if (elaspedTime >= time)
    {
        autoStage++;
        elaspedTime = -1.0;
    }
}
void Autonomous::AccelProfile(double time, double step)
{
    if (elaspedTime == -1.0)
    {
        countOffset = mainRobot->GetLoopCount();
        xdot = 0.0;
    }
    elaspedTime = (mainRobot->GetLoopCount() - countOffset) * LOOPTIME;
    xdot += step;
    if (elaspedTime >= time)
    {
        autoStage++;
        elaspedTime = -1.0;
    }
}
void Autonomous::ConstProfile(double time, double vel)
{
    if (elaspedTime == -1.0)
    {
        countOffset = mainRobot->GetLoopCount();
        xdot = 0.0;
    }
    elaspedTime = (mainRobot->GetLoopCount() - countOffset) * LOOPTIME;
    xdot = vel;
    if (elaspedTime >= time)
    {
        autoStage++;
        elaspedTime = -1.0;
    }
}

void Autonomous::AccelProfileDynamic(double time, double xStep, double yStep, double psiStep)
{
    if (elaspedTime == -1.0)
    {
        countOffset = mainRobot->GetLoopCount();
    }
    elaspedTime = (mainRobot->GetLoopCount() - countOffset) * LOOPTIME;
    xdot += xStep;
    ydot += yStep;
    psidot += psiStep;
    if (elaspedTime >= time)
    {
        elaspedTime = -1.0;
        autoStage++;
    }
}
void Autonomous::ConstProfileDynamic(double time, double xVel, double yVel, double psiVel)
{
    if (elaspedTime == -1.0)
    {
        countOffset = mainRobot->GetLoopCount();
    }
    elaspedTime = (mainRobot->GetLoopCount() - countOffset) * LOOPTIME;
    xdot = xVel;
    ydot = yVel;
    psidot = psidot;
    if (elaspedTime >= time)
    {
        elaspedTime = -1.0;
        autoStage++;
    }
}
/**
 * @brief Trajectory acceleration profile for autonomous
 *
 * @param stageTime time to complete stage
 * @param vxStart initial x vel
 * @param vyStart initial y vel
 * @param vpsiStart initial psi vel
 * @param vxStop final x vel
 * @param vyStop final y vel
 * @param vpsiStop final psi vel
 */
void Autonomous::TrajAccelProfile(double stageTime, double vxStart, double vyStart, double vpsiStart, double vxStop, double vyStop, double vpsiStop)
{
    double k;
    // when first called, created the offset
    if (elaspedTime == -1.0)
    {
        countOffset = mainRobot->GetLoopCount();
    }
    // propagate time
    elaspedTime = (mainRobot->GetLoopCount() - countOffset) * LOOPTIME;

    // percent of profile completed
    k = (elaspedTime / stageTime);

    // lineraly interpolate between start and stop
    xdot = vxStart + k * (vxStop - vxStart);
    ydot = vyStart + k * (vyStop - vyStart);
    psidot = vpsiStart + k * (vpsiStop - vpsiStart);

    // if time is up, increment stage
    if (elaspedTime > stageTime - (LOOPTIME / 2.0))
    {
        elaspedTime = -1.0;
        autoStage++;
    }
}
/**
 * @brief Trajectory constant profile for autonomous
 *
 * @param stageTime time to complete stage
 * @param vx constant x vel
 * @param vy constant y vel
 * @param vpsi constant psi vel
 */
void Autonomous::TrajConstProfile(double stageTime, double vx, double vy, double vpsi)
{
    // call TrajAccelProfile with start and stop vel the same
    TrajAccelProfile(stageTime, vx, vy, vpsi, vx, vy, vpsi);
}
