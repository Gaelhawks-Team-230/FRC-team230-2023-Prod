#include "TalonXXV.h"
#include "frc/RobotController.h"
#include <fmt/core.h>
#include "Common.h"
#include <units/time.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

TalonXXV::TalonXXV() : TimedRobot(units::second_t LOOPTIME)
{
  // printf("Creating drivetrain\n");
  m_drivetrain = new Drivetrain(this);
  // printf("Drivetrain created\n");
  // printf("Creating arm and grabber\n");
  m_arm = new Arm(this);
  // printf("Arm created\n");
  m_grabber = new Grabber(this);
  // printf("grabber created\n");
  // printf("Creating joystick\n");
  m_userInput = new Joystick(this);
  // printf("Joystick created\n");
  // printf("Creating pdp\n");
  m_pdp = new frc::PowerDistribution(PDP_MODULE, frc::PowerDistribution::ModuleType::kCTRE);
  // printf("PDP created\n");
  // printf("Creating auto\n");
  m_autonomous = new Autonomous(this);
  // printf("Auto created\n");
  // printf("Creating vision\n");
  m_vision = new Vision(this);
  // printf("Vision created\n");

  m_ledPanel = new LEDPanel(this);

  LocalReset();
}

void TalonXXV::RobotInit()
{
  LocalReset();
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoPathA, kAutoPathA);
  m_chooser.AddOption(kAutoPathB, kAutoPathB);
  m_chooser.AddOption(kAutoPathC, kAutoPathC);
  m_chooser.AddOption(kAutoPathD, kAutoPathD);
  m_chooser.AddOption(kAutoPathE, kAutoPathE);
  m_chooser.AddOption(kAutoPathF, kAutoPathF);
  m_chooser.AddOption(kAutoPathH, kAutoPathH);
  // m_chooser.AddOption(kAutoPathI, kAutoPathI);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  m_limelight_chooser.SetDefaultOption(kPipelineDefault, kPipelineDefault);
  m_limelight_chooser.AddOption(kPipelineAprilTag, kPipelineAprilTag);
  frc::SmartDashboard::PutData("Limelight Pipeline", &m_limelight_chooser);
}
void TalonXXV::LocalReset()
{
  // * Global loopcount
  m_count = 0;
  m_alliance = frc::DriverStation::GetAlliance();
  wrist_bias_increase_loopcount = 0;
  wrist_bias_decrease_loopcount = 0;
  m_drivetrain->LocalReset();
  m_userInput->LocalReset();
  m_arm->LocalReset();
  m_grabber->LocalReset();
  m_ledPanel->LocalReset();
  m_autonomous->LocalReset();
  m_vision->LocalReset();
  button_x = false;
  button_x_z = false;
  button_x_pressed = false;
  button_x_released = false;
  autoScoreStage = 0;
  autoScoreCountOffset = 0;
}
/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void TalonXXV::RobotPeriodic()
{

  if (frc::DriverStation::IsEnabled())
  {
    return;
  }
  else if (frc::DriverStation::IsDSAttached())
  {
    m_ledPanel->DisplayConnected();
  }
  else
  {
    m_ledPanel->DisplayDisconnected();
  }

  m_ledPanel->Service();
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void TalonXXV::AutonomousInit()
{
  LocalReset();
  m_grabber->SetWristAngle(80.0);
  m_drivetrain->GyroReset();
  m_drivetrain->EnableGyro();
  m_autoSelected = m_chooser.GetSelected();

  m_pipelineSelected = m_limelight_chooser.GetSelected();
  if (m_pipelineSelected == kPipelineAprilTag)
  {
    m_vision->SetCameraMode(APRIL_TAG_CAM);
  }
  else
  {
    m_vision->SetCameraMode(DRIVER_CAM);
  }
  // m_arm->SetGoalPos(STOW_POS);
  // m_vision->SetTargetFilter(TargetFilter::BEST_TARGET);
  // m_vision->SetVisionFrame(VisionFrame::ROBOT_FRAME);
  // m_vision->SetCamera(Camera::LEFT_CAMERA);
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);

  fmt::print("Auto selected: {}\n", m_autoSelected);
  fmt::print("Limelight Pipeline selected: {}\n", m_pipelineSelected);
}

void TalonXXV::AutonomousPeriodic()
{
  m_count++;
  double xdot, ydot, psidot;

  m_drivetrain->Analyze();
  m_arm->Analyze();
  m_grabber->Analyze();
  m_vision->Analyze();

  if (m_autoSelected == kAutoPathA)
  {
    // Custom Auto goes here
    // if (!trapizoidPlanner->CheckRoutineStatus()){
    //   trapizoidPlanner->Service(&xdot);
    // }
    m_autonomous->PathA();
  }
  else if (m_autoSelected == kAutoPathB)
  {
    m_autonomous->PathB();
  }
  else if (m_autoSelected == kAutoPathC)
  {
    m_autonomous->PathC();
  }
  else if (m_autoSelected == kAutoPathD)
  {
    m_autonomous->PathD();
  }
  else if (m_autoSelected == kAutoPathE)
  {
    m_autonomous->PathE();
  }
  else if (m_autoSelected == kAutoPathF)
  {
    m_autonomous->PathF();
  }
  else if (m_autoSelected == kAutoPathH)
  {
    m_autonomous->PathH();
  }
  else if (m_autoSelected == kAutoPathI)
  {
    m_autonomous->PathI();
  }
  else
  {
    // Default Auto goes here
  }
  xdot = m_autonomous->GetAutoXDot();
  ydot = m_autonomous->GetAutoYDot();
  psidot = m_autonomous->GetAutoPsidot();

  if (m_autoSelected == kAutoPathD || m_autoSelected == kAutoPathE || m_autoSelected == kAutoPathF || m_autoSelected == kAutoPathH || m_autoSelected == kAutoPathI)
  {
    m_drivetrain->ToFieldCoordinates(&xdot, &ydot);
  }
  m_drivetrain->DriveControl(xdot, ydot, psidot);
  m_arm->Service();
  m_grabber->Service();
  ServiceDash();
}

void TalonXXV::TeleopInit()
{
  LocalReset();
  // m_vision->SetTargetFilter(TargetFilter::BEST_TARGET);
  // m_vision->SetFiducialTargetLock(1);
  m_pipelineSelected = m_limelight_chooser.GetSelected();
  if (m_pipelineSelected == kPipelineAprilTag)
  {
    m_vision->SetCameraMode(APRIL_TAG_CAM);
  }
  else
  {
    m_vision->SetCameraMode(DRIVER_CAM);
  }
  m_arm->SetGoalPos(STOW_POS);
  m_vision->SetVisionFrame(VisionFrame::ROBOT_FRAME);
}

void TalonXXV::TeleopPeriodic()
{
  double xdot, ydot, psidot;
  double vxBeep, vyBeep;
  m_count++;

  // Get sensor data
  m_drivetrain->Analyze();
  m_userInput->Analyze();
  m_arm->Analyze();
  m_grabber->Analyze();
  m_vision->Analyze();

  // * Drive train control
  xdot = m_userInput->GetFlightCtrl_CMD_X();
  ydot = m_userInput->GetFlightCtrl_CMD_Y();
  psidot = m_userInput->GetFlightCtrl_CMD_R();

  // check Beep right button on flight ctrt
  if (m_userInput->GetFlightCtrlButton(BEEP_FOREWARD_BUTTON) == kPressing)
  {
    vxBeep = 2.0 / LOOPTIME;
  }
  else if (m_userInput->GetFlightCtrlButton(BEEP_BACKWARD_BUTTON) == kPressing)
  {
    vxBeep = -2.0 / LOOPTIME;
  }
  else
  {
    vxBeep = 0.0;
  }
  xdot += vxBeep;
  if (m_userInput->GetFlightCtrlButton(BEEP_RIGHT_BUTTON) == kPressing)
  {
    vyBeep = 2.0 / LOOPTIME;
  }
  else if (m_userInput->GetFlightCtrlButton(BEEP_LEFT_BUTTON) == kPressing)
  {
    vyBeep = -2.0 / LOOPTIME;
  }
  else
  {
    vyBeep = 0.0;
  }
  ydot += vyBeep;

  // * Drive control convert to field right off the bat
  if (m_userInput->GetFlightCtrlButton(FIELD_ROBOT_SWITCH))
  {
    m_drivetrain->SetZeroVelDebugModeOff();
  }
  else
  {
    m_drivetrain->ToFieldCoordinates(&xdot, &ydot);
    m_drivetrain->SetZeroVelDebugModeOff();
  }
  // switch (m_userInput->GetGamepadButton(AUTO_BALANCE_BUTTON))
  // {
  // case kOff:
  //   break;

  // case kPressing:
  //   m_drivetrain->PitchGyroReset();
  //   break;

  // case kHeld:
  //   m_drivetrain->AutoBalance(&xdot);
  //   break;
  // }
  //* Grabber - intake
  // m_userInput->GetGamepadButton(COLLECT_CONE_BUTTON) && !(m_userInput->GetGamepadButton(TOP_ROW_BUTTON))
  if (m_userInput->GetGamepadButton(COLLECT_CONE_BUTTON))
  {
    m_grabber->GatherCone();
    if (m_userInput->GetGamepad_RAW_Y() < -0.5)
    {
      m_arm->ClearNoseMode();
      m_arm->SetGoalPos(REARPICKUP_POS);
    }
    else
    {
      m_arm->ClearNoseMode();
      m_arm->SetGoalPos(PICKUP_POS);
    }
  }

  else if (m_userInput->GetGamepadButton(COLLECT_CUBE_BUTTON))
  {
    m_grabber->GatherCube();

    // * collecting cube from substation
    if (m_userInput->GetDpadUpButton())
    {
      m_arm->SetGoalPos(SUB_POS);
    }
    else if (m_userInput->GetGamepad_RAW_Y() < -0.5)
    {
      m_arm->SetGoalPos(REARPICKUP_POS);
    }
    else
    {
      m_arm->SetGoalPos(PICKUP_POS);
    }
  }

  else
  {
    m_grabber->SetHoldState(false);
    m_grabber->StopRollers();
  }

  // * collecting cone from substation
  if (m_userInput->GetGamepadButton(SUB_OPEN_BUTTON))
  {
    m_arm->SetNoseMode();
    m_arm->SetGoalPos(SUB_POS);
    m_grabber->OpenJaw();
  }

  // if ((m_userInput->GetGamepadButton(SUB_BIAS_CLOSE_BUTTON) == kPressing) || (m_userInput->GetGamepadButton(SUB_BIAS_CLOSE_BUTTON) == kHeld))
  //  printf("Button state : %d\n", m_userInput->GetGamepadButton(SUB_BIAS_CLOSE_BUTTON));
  if (m_userInput->GamepadBtnPushed(SUB_BIAS_CLOSE_BUTTON))
  {
    m_arm->BiasArm(-8.0);
    m_grabber->GatherCone();
  }
  if (m_userInput->GetGamepadButton(SUB_BIAS_CLOSE_BUTTON) == kReleasing)
  {
    m_arm->BiasArm(0.0);
  }

  if (m_userInput->GetGamepadButton(EJECT_CONE_BUTTON))
  {
    if (m_arm->GetGoalPos() == LOW_POS)
    {
      m_grabber->EjectConeFaster();
    }
    else
    {
      m_grabber->EjectCone();
    }
    // Move backwards when ejecting
    // xdot-= 30.0;
  }
  else if (m_userInput->GetGamepadButton(EJECT_CUBE_BUTTON))
  {
    if (m_arm->GetGoalPos() == LOW_POS)
    {
      m_grabber->EjectCube();
      // * previously used for full faster cmd
    }
    else
    {
      m_grabber->EjectCube();
    }
  }

  // * Motion position
  // if (m_userInput->GetDpadLeftButton())
  // {
  //   m_arm->SetGoalPos(PICKUP_POS);
  //   // printf("pressed left button\n");
  // }

  // * checking for dpad down pressed, changing bias on transition of button depending on pressed / released
  // (m_userInput->GetGamepad_RAW_R() < -0.5)
  button_x = m_userInput->GetDpadDownPushed();
  button_x_pressed = (button_x && !button_x_z) ? true : false;
  button_x_released = (!button_x && button_x_z) ? true : false;
  button_x_z = button_x;
  if (button_x_pressed)
  {
    m_arm->BiasArm(-8.0);
  };
  if (button_x_released)
  {
    m_arm->BiasArm(0.0);
  };

  switch (m_userInput->GetFlightCtrlButton(WRIST_BIAS_INCREASE))
  {
  case kOff:
    wrist_bias_increase_loopcount = 0;
    break;

  case kPressing:
    wrist_bias_increase_loopcount = 0;
    break;

  case kHeld:
    wrist_bias_increase_loopcount++;
    break;

  case kReleasing:
    wrist_bias_increase_loopcount = 0;
    break;
  }
  // if button pressed for more than 1 second, increase wrist angle bias
  if (wrist_bias_increase_loopcount > (1.0 / LOOPTIME))
  {
    m_grabber->IncreaseWristAngleBias();
    wrist_bias_increase_loopcount = 0;
  }

  switch (m_userInput->GetFlightCtrlButton(WRIST_BIAS_DECREASE))
  {
  case kOff:
    wrist_bias_decrease_loopcount = 0;
    break;

  case kPressing:
    wrist_bias_decrease_loopcount = 0;
    break;

  case kHeld:
    wrist_bias_decrease_loopcount++;
    break;

  case kReleasing:
    wrist_bias_decrease_loopcount = 0;
    break;
  }
  // if button pressed for more than 1 second, decrease wrist angle bias
  if (wrist_bias_decrease_loopcount > (1.0 / LOOPTIME))
  {
    m_grabber->DecreaseWristAngleBias();
    wrist_bias_decrease_loopcount = 0;
  }

  if (m_userInput->GetGamepad_RAW_R() < -0.5)
  {
    if (m_arm->IsNoseMode())
    {
      m_grabber->EjectConeFaster();
    }
    else
    {
      m_grabber->EjectCone();
      // Move backwards when ejecting
      xdot -= 30.0;
    }
  }

  if (m_userInput->GetGamepadButton(STOW_POS_BUTTON))
  {
    m_arm->SetGoalPos(STOW_POS);
    // printf("pressed stow button\n");
  }

  // m_userInput->GetGamepadButton(TOP_ROW_BUTTON) && !(m_userInput->GetGamepadButton(COLLECT_CONE_BUTTON))
  if (m_userInput->GetGamepadButton(TOP_ROW_BUTTON))
  {
    m_arm->SetGoalPos(HIGH_POS);
    // printf("pressed top button\n");
  }
  if (m_userInput->GetGamepadButton(MIDDLE_ROW_BUTTON))
  {
    m_arm->SetGoalPos(MID_POS);
  }
  if (m_userInput->GetGamepadButton(BOTTOM_ROW_BUTTON))
  {
    m_arm->SetGoalPos(LOW_POS);
  }

  // setting LED panel
  if (m_userInput->GetDpadRightPushed())
  {
    m_ledPanel->DisplayCone();
  }
  if (m_userInput->GetDpadLeftPushed())
  {
    m_ledPanel->DisplayCube();
  }
  // * Vision Stuff
  if (m_userInput->GetFlightCtrlButton(VISION_CENTER_BUTTON))
  // if (m_userInput->GetFlightCtrlButton(VISION_CENTER_BUTTON)) // center
  {
    m_vision->SetMiddleGoalPos();
    m_vision->DriveTargetting(&xdot, &ydot, &psidot);
  }
  else if (m_userInput->GetFlightCtrlButton(VISION_LEFT_BUTTON)) // left target in field frame
  {
    m_vision->SetRightGoalPos();
    m_vision->DriveTargetting(&xdot, &ydot, &psidot);
  }
  // else if (m_userInput->GetFlightCtrlButton(VISION_RIGHT_BUTTON))
  // {
  //   // m_vision->SetRightGoalPos();
  //   autoScoreStage = 1;
  // }
  // else if (m_userInput->GetFlightCtrlButton(VISION_LEFT_BUTTON))
  // {
  //   // m_vision->SetLeftGoalPos();
  //   autoScoreStage = 1;
  // }
  // else if (m_userInput->GetFlightCtrlButton(DATA_DEBUG_BUTTON))
  // {
  //   m_vision->SetDataGoalPos();
  //   m_vision->DriveTargetting(&xdot, &ydot, &psidot);
  //   xdot = 0.0;
  //   ydot = 0.0;
  //   psidot = 0.0;
  // }

  // goal_pos.Set(0.0, 0.0, 180.0);
  // m_vision->SetGoalPos(goal_pos);
  // m_vision->DriveTargetting(&xdot, &ydot, &psidot);
  // double elaspedTime;
  // switch (autoScoreStage)
  // {
  // case 0:
  //   //  DO NOTHING
  //   break;
  // case 1:
  //   autoScoreCountOffset = m_count;
  //   autoScoreStage++;
  //   break;
  // case 2:
  //   if (m_vision->TargetFound() < 0.5)
  //   {
  //     break;
  //   }
  //   m_vision->DriveTargetting(&xdot, &ydot, &psidot);
  //   elaspedTime = (m_count - autoScoreCountOffset) * LOOPTIME;
  //   if (elaspedTime >= 2.0)
  //   {
  //     autoScoreStage = 0;
  //   }
  //   break;
  // }

  // * Begin Drive control
  if (m_userInput->GetFlightCtrlButton(RESET_GYRO_BUTTON))
  {
    m_drivetrain->GyroReset();
  }

  if (m_userInput->GetFlightCtrlButton(GYRO_BUTTON_SWITCH))
  {
    m_drivetrain->EnableGyro();
  }
  else
  {
    m_drivetrain->DisableGyro();
  }
  if (m_userInput->GetFlightCtrlButton(GRID_ALIGN_BUTTON_1) || m_userInput->GetFlightCtrlButton(GRID_ALIGN_BUTTON_2))
  {
    double pErr = GRID_ALIGN_ANGLE - Wrap(m_drivetrain->GetGyroReading());
    double vCmd = pErr * GYRO_ANGLE_KP;
    psidot = Limit(-120.0, 120.0, vCmd);
  }

  m_drivetrain->DriveControl(xdot, ydot, psidot);
  m_arm->Service();
  m_grabber->Service();
  m_ledPanel->Service();
  ServiceDash();
}
void TalonXXV::DisabledInit() {}

void TalonXXV::DisabledPeriodic() {}

void TalonXXV::TestInit() {}

void TalonXXV::TestPeriodic() {}

void TalonXXV::SimulationInit() {}

void TalonXXV::SimulationPeriodic() {}

void TalonXXV::ServiceDash()
{
  m_drivetrain->UpdateDash();
  m_userInput->UpdateDash();
  m_arm->UpdateDash();
  m_grabber->UpdateDash();
  m_vision->UpdateDash();

  frc::SmartDashboard::PutNumber("Loop count", m_count);
  frc::SmartDashboard::PutNumber("CAN pct utilization", (frc::RobotController::GetCANStatus().percentBusUtilization * 100));
  frc::SmartDashboard::PutBoolean("Alliance", m_alliance);
}
/**
 * @brief Wrap value in degrees
 *
 * @param val value in degrees
 * @return double wrapped value in degrees
 */
double TalonXXV::Wrap(double val)
{
  double rVal = val * (M_PI / 180);
  double rWrap = atan2(sin(rVal), cos(rVal));
  return rWrap * (180 / M_PI);
}
/**
 * @brief Limit the value between a range
 *
 * @param min minimum value
 * @param max maximum value
 * @param val value to compare
 * @return double
 */
double TalonXXV::Limit(double min, double max, double val)
{
  if (val > max)
  {
    return max;
  }
  if (val < min)
  {
    return min;
  }
  return val;
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<TalonXXV>();
}
#endif
