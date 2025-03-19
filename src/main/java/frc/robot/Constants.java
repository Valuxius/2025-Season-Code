// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class RobotConstants { 
    public static final double kMaxSpeedMetersPerSecond = 3; //4.3 (recommended max from Jackie: 3)
    public static final double kMaxAccelerationMetersPerSecond = 3.5; // 4.3 theoretical max
    public static final double kMaxAngularSpeed = 2 * Math.PI; //radians per sec 
    public static final double kMaxAngularAcceleration = 2 * Math.PI; // radians per second per second

    public static final double kWheelDiameterMeters = 0.0762; //diameter of the wheels in meters
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI; //0.239, circumference of the wheels in meters
    public static final double kDriveMotorReduction = (45.0 * 22) / (13 * 15); //5.079, factor for how much slower wheel is compared to motor
    public static final double kDriveMotorFreeSpeedRps = 5676 / 60; //94.0, max rotations per second for drive motor
    public static final double kDriveWheelFreeSpeedRps = (kDriveMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDriveMotorReduction; //4.43, max speed for drive wheel
    public static final double kDriveEncoderPositionFactor = kWheelCircumferenceMeters / kDriveMotorReduction; // meters
    public static final double kDriveEncoderVelocityFactor = (kWheelCircumferenceMeters / kDriveMotorReduction) / 60.0; // meters per second

    //gains for drive motor PID (defaults)
    public static final double kDriveP = 0.05; 
    public static final double kDriveI = 0; 
    public static final double kDriveD = 0.005; 
    public static final double kDriveFF = 1 / kDriveWheelFreeSpeedRps; 
    public static final double kDriveMinOutput = -1; 
    public static final double kDriveMaxOutput = 1; 

    //gains for turn motor PID (defaults)
    public static final double kTurnP = 0.8; 
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0.0;
    public static final double kTurnFF = 0;
    public static final double kTurnMinOutput = -1;
    public static final double kTurnMaxOutput = 1;

    //gains for climb motor PID (defaults)
    public static final double kClimbP = 0.2;
    public static final double kClimbI = 0.003;
    public static final double kClimbD = 0;

    //gains for elevator motor PID (defaults)
    public static final double kElevatorP = 0;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double kElevatorS = 0;
    public static final double kElevatorG = 0.025;
    public static final double kElevatorV = 1.0 / 15.0;
    public static final double kElevatorA = 0;

    //gains for shooter motor PID (defaults)
    public static final double kShooterP = 0.005;
    public static final double kShooterI = 0.004;
    public static final double kShooterD = 0;

    //drive motor ports 
    public static final int kFrontLeftDriveMotorPort = 4;
    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kBackLeftDriveMotorPort = 6;
    public static final int kBackRightDriveMotorPort = 8;

    public static final int kFrontLeftTurnMotorPort = 3;
    public static final int kFrontRightTurnMotorPort = 1;
    public static final int kBackLeftTurnMotorPort = 5;
    public static final int kBackRightTurnMotorPort = 7;

    //elevator motor ports
    public static final int kLeftElevatorMotorPort = 13;
    public static final int kRightElevatorMotorPort = 12;

    //shooter motor ports
    public static final int kShooterMotorPort = 10;
    public static final int kRotationMotorPort = 9;
  
    //climb motor port
    public static final int kClimbMotorPort = 11;

    //usb port for drive controller 
    public static final int kDriverControllerPort = 0;

    //important buttons/axis for driver controller 
    public static final int kLeftXAxisPort = 0;
    public static final int kLeftYAxisPort = 1;
    public static final int kRightXAxisPort = 4;

    public static final int kDriverLeftTriggerAxis = 2; 
    public static final int kDriverRightTriggerAxis = 3;   

    public static final int kDriverXButton = 3; 
    public static final int kDriverAButton = 1; 
    public static final int kDriverBButton = 2; 
    public static final int kDriverYButton = 4; 
    public static final int kDriverLeftShoulder = 5;
    public static final int kDriverRightShoulder = 6; 
    public static final int kDriverMinusButton = 7;
    public static final int kDriverPlusButton = 8;

    //usb port for manipulator controller
    public static final int kManipulatorControllerPort = 1;

    //important buttons/axis for manipulator controller
    public static final int kManipulatorLeftTriggerAxis = 2; 
    public static final int kManipulatorRightTriggerAxis = 3; 

    public static final int kManipulatorXButton = 3; 
    public static final int kManipulatorAButton = 1; 
    public static final int kManipulatorBButton = 2; 
    public static final int kManipulatorYButton = 4; 
    public static final int kManipulatorLeftShoulder = 5;
    public static final int kManipulatorRightShoulder = 6; 
    public static final int kManipulatorMinusButton = 7;
    public static final int kManipulatorPlusButton = 8;
    public static final int kManipulatorLeftJoystick = 9;
    public static final int kManipulatorRightJoystick = 10;


    //shift in motors to temporarily test robot without having to recalibrate
    //set to 0 after calibration
    public static final double kFrontLeftOffset = 0;
    public static final double kBackLeftOffset = 0;
    public static final double kFrontRightOffset = 0;
    public static final double kBackRightOffset = 0;

    //POV angles for the DPad
    public static final int kPOVUp = 0;
    public static final int kPOVRight = 90;
    public static final int kPOVDown = 180;
    public static final int kPOVLeft = 270;

    //slew rate helps stop wheel failure
    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    //distance from center of left wheel to right wheel
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    //distance from center of front wheel to back wheel
    public static final double kTrackLength = Units.inchesToMeters(24.5);

    //angle offsets for each motor (in radians)
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2 + Units.degreesToRadians(kFrontLeftOffset);
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    //kinematics class that generates swerve module states from chassis Speeds
    public static final SwerveDriveKinematics kDriveKinematics =
    new SwerveDriveKinematics(
      new Translation2d(kTrackLength / 2, kTrackWidth / 2), //front left
      new Translation2d(kTrackLength / 2, -kTrackWidth / 2), //front right      
      new Translation2d(-kTrackLength / 2, kTrackWidth / 2), //rear left
      new Translation2d(-kTrackLength / 2, -kTrackWidth / 2)); //rear right

    //robot clock cycle 
    public static final double kDriverPeriod = TimedRobot.kDefaultPeriod;

    public static final double kTurnEncoderPositionFactor = 2 * Math.PI; //radians 
    public static final double kTurnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurnEncoderPositionPIDMinInput = 0; //min input for turn encoder
    public static final double kTurnEncoderPositionPIDMaxInput = 2 * Math.PI; //max input for turn encoder

    //current limits for swerve motors
    public static final int kDriveCurrentLimit = 40;
    public static final int kTurnCurrentLimit = 20;

  }
}
