#pragma once


#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "frc/DriverStation.h"
#include "frc/PowerDistribution.h"

#include "Autonomous.h"
#include "Drivetrain.h"
#include "Joystick.h"
#include <Loggers.h>
#include "Arm.h"
#include "Grabber.h"
#include "Vision.h"
#include "LEDPanel.h"

class TalonXXV : public frc::TimedRobot
{
public:
  TalonXXV();

  frc::PowerDistribution *m_pdp;

  Joystick *m_userInput;
  Drivetrain *m_drivetrain;
  Arm *m_arm;
  Grabber *m_grabber;
  Vision *m_vision;

  LEDPanel *m_ledPanel;
  Autonomous *m_autonomous;
  CLoggers *m_mainRobot;

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
  void ServiceDash();
  void LocalReset();

  unsigned int GetLoopCount() { return m_count; };
  frc::DriverStation::Alliance GetAlliance() { return m_alliance; };
  static double Wrap(double val);
  static double Limit(double min, double max, double val);

private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Do Nothing";
  const std::string kAutoPathA = "Path A: Cone Balance via Charge";
  const std::string kAutoPathB = "Path B: Place Cone";
  const std::string kAutoPathC = "Path C: Place Cone and Exit";
  const std::string kAutoPathD = "Path D: Cone and Cube";
  const std::string kAutoPathE = "Path E: Cone, Cube and Balance via Charge";
  const std::string kAutoPathF = "Path F: Cone, Cube and Cone";
  const std::string kAutoPathH = "Path H: Cone, Cube and Cone | Cable P";
  const std::string kAutoPathI = "Path I: NoseMode Cone, Cube and Balance";
  
  std::string m_autoSelected;

  frc::SendableChooser<std::string> m_limelight_chooser;
  const std::string kPipelineDefault = "Driver Cam";
  const std::string kPipelineAprilTag = "April Tag Tracking";

  std::string m_pipelineSelected;

  unsigned int m_count;
  // kBlue or kRed
  frc::DriverStation::Alliance m_alliance;

  frc::Compressor *m_pcm_compressor;

  unsigned int wrist_bias_increase_loopcount;
  unsigned int wrist_bias_decrease_loopcount;

  bool button_x_z;
  bool button_x;
  bool button_x_pressed;
  bool button_x_released;

  unsigned int autoScoreStage;
  unsigned int autoScoreCountOffset;
};