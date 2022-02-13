#pragma once

#include <vector>
#include "controllers/Controllers.h"

#define __CONTROLMAP_USING_JOYSTICK__ false

using namespace wml;
using namespace wml::controllers;

struct ControlMap {
  static void InitSmartControllerGroup(wml::controllers::SmartControllerGroup &contGroup) {
  //remap Here (map POV buttons to names ect)
  }

  // ------------------ Values ------------------

  // Controllers
  static constexpr int JoystickPort = 0;
  static constexpr int XboxPort = 1;

  // USB port numbers
  static const int Driver = 1;
  static const int CoDriver = 2;

  // Deadzone
  static constexpr double XboxDeadzone = 0.15;
  static constexpr double TriggerDeadzone = 0.05;

  // PCM1
  static constexpr int PCModule = 9;
  static constexpr int PressureSensorPort = 0;
  static constexpr int CompressorPort = 0;

  // Drivetrain
  static constexpr double trackWidth = 0.56;
  static constexpr double trackDepth = 0.60;
  static constexpr double wheelRadius = 0.0762; 
  static constexpr double mass = 50;

  static constexpr int leftMotorPortF = 99;
  static constexpr int leftMotorPortB = 99;
  static constexpr int rightMotorPortF = 99;
  static constexpr int rightMotorPortB = 99;

  static constexpr double maxDrivetrainPower = 0.5;

  // Intake 1
  static constexpr int intake1MotorPort1 = 99;
  static constexpr int intake1MotorPort2 = 99;

  // Intake 2
  static constexpr int intake2MotorPort1 = 99;
  static constexpr int intake2MotorPort2 = 99;

  // Shooter

  static constexpr int shooterMotorPort1 = 99;
  static constexpr int shooterMotorPort2 = 99;
  static constexpr int indexingMotorPort1 = 99;
  static constexpr int indexingMotorPort2 = 99;

  // Climber

  // Example Elevator
  static constexpr int ElevatorMotorPort = 99;
  static constexpr int ElevatorSolenoidPort = 99;
  static constexpr bool ElevatorToggle = false;
  static constexpr bool ReverseElevatorToggle = false;

  // ------------------ Controls ------------------

  // Drivetrain
  inline static const wml::controllers::tAxis sideControl{ Driver, wml::controllers::Joystick::kXAxis };
  inline static cosnt wml::controllers::tAxis forwardControl{ Driver, wml::controllers::Joystick::kYAxis };

  // Intake 1
  inline static const wml::controllers::tAxis intake1Control{ CoDriver, wml::controllers::XboxController::kLeftYAxis };
  inline static const wml::controllers::tButton intake1Actuation{ CoDriver, wml::controllers::XboxController::kX };

  // Intake 2
  inline static const wml::controllers::tAxis intake2Control{ CoDriver, wml::controllers::XboxController::kRightYAxis };
  inline static const wml::controllers::tButton intake2Actuation{ CoDriver, wml::controllers::XboxController::kB };

  // Shooter
  inline static const wml::controllers::tButton shooterSwitch{ CoDriver, XboxController::kA };
  inline static const wml::controllers::tAxis shooterShoot{ CoDriver, XboxController::kRightThrottle }; //haha shootershoot funny

  // Example Elevator
  inline static const wml::controllers::tAxis Belevator{ CoDriver, XboxController::kLeftYAxis };
  inline static const wml::controllers::tButton BelevatorActuation{ CoDriver, XboxController::kY };
  inline static const wml::controllers::tButton BelevatorToggle{ CoDriver, XboxController::kX };
};