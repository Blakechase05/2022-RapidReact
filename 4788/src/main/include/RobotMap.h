#pragma once 

// General
#include <string>
#include <stdint.h>


// FRC/WPI
#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/DoubleSolenoid.h>
#include <frc/GenericHID.h>

#include <cameraserver/CameraServer.h>
#include <frc/DriverStation.h>

#include <frc/SpeedControllerGroup.h>
#include <frc/PowerDistribution.h>
#include <frc/Servo.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogInput.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/SmallString.h>

// WML sucks
#include <WMLCtre.h>
#include <controllers/Controllers.h>
#include <actuators/BinaryServo.h>
#include <actuators/Compressor.h>
#include <NTProvider.h>
#include <actuators/DoubleSolenoid.h>
#include <actuators/VoltageController.h>
#include <Drivetrain.h>
#include <sensors/Encoder.h>
#include <sensors/LimitSwitch.h>
#include <sensors/NavX.h>
#include <sensors/PressureSensor.h>
#include <control/PIDController.h>
#include <MotionProfiling.h>
#include <Toggle.h>

#include <devices/StateDevice.h>
#include <strategy/StrategyController.h>
#include <strategy/MPStrategy.h>
#include <control/MotorFilters.h>
#include <Gearbox.h>
#include <strategy/Strategy.h>
#include <sensors/BinarySensor.h>

// WML Rev
#include <WMLRev.h>

// Local Files
#include "ControlMap.h"

struct RobotMap {
  // Controllers
  wml::controllers::XboxController xbox{ ControlMap::XboxPort };
  wml::controllers::Joystick joystick{ ControlMap::JoystickPort };
  wml::controllers::SmartControllerGroup contGroup{ xbox, joystick};

  struct ControlSystem {
    wml::sensors::PressureSensor pressureSensor{ ControlMap::PressureSensorPort };
    wml::actuators::Compressor compressor{ ControlMap::CompressorPort, wml::actuators::PneumaticsModuleType::kCTRE, "Cj" };
  }; ControlSystem controlSystem;

  struct ExampleElevatorSystem {
    wml::TalonSrx elevatorMotor{ControlMap::ElevatorMotorPort, 2048};
    wml::actuators::DoubleSolenoid elevatorSolenoid{ ControlMap::PCModule, ControlMap::ElevatorSolenoidPort, 0.1};
  }; ExampleElevatorSystem exampleElevatorSystem;

  struct DrivebaseSystem {
    // Init Motors
    wml::TalonSrx leftMotorF{ControlMap::leftMotorPortF, 2048};
    wml::TalonSrx leftMotorB{ControlMap::leftMotorPortB, 2048};

    wml::TalonSrx rightMotorF{ControlMap::rightMotorPortF, 2048};
    wml::TalonSrx rightMotorB{ControlMap::rightMotorPortB, 2048};

    wml::actuators::MotorVoltageController leftMotors = wml::actuators::MotorVoltageController::Group(leftMotorF, leftMotorB);
    wml::actuators::MotorVoltageController rightMotors = wml::actuators::MotorVoltageController::Group(rightMotorF, rightMotorB);

    wml::Gearbox LGearbox{&leftMotors, &leftMotorF};
    wml::Gearbox RGearbox{&rightMotors, &rightMotorF};
    
    wml::sensors::NavX navX{};
    wml::sensors::NavXGyro gyro{navX.Angular(wml::sensors::AngularAxis::YAW)};

    wml::DrivetrainConfig drivetrainConfig{LGearbox, RGearbox, &gyro, ControlMap::trackWidth, ControlMap::trackDepth, ControlMap::wheelRadius, ControlMap::mass};
    wml::control::PIDGains gainsVelocity{"Drivetrain Velocity", 1};
    wml::Drivetrain drivetrain{drivetrainConfig, gainsVelocity};

  }; DrivebaseSystem drivebaseSystem;

  struct Intake1System {
    wml::TalonSrx intake1Motor1{ControlMap::intake1MotorPort1, 2048};
    wml::TalonSrx intake1Motor2{ControlMap::intake1MotorPort2, 2048};

  }; Intake1System intake1System;

  struct Intake2System {
    wml::TalonSrx intake2Motor1{ControlMap::intake2MotorPort1, 2048};
    wml::TalonSrx intake2Motor2{ControlMap::intake2MotorPort2, 2048};
  }; Intake2System intake2System;

};