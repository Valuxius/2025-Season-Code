// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.Net;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.utils.AnalogTrigger;
import frc.robot.utils.LimelightHelpers;

public class RobotContainer {
  //creating the subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem(
    RobotConstants.kLeftElevatorMotorPort, 
    RobotConstants.kRightElevatorMotorPort);
  private final ShooterSubsystem m_shooter = new ShooterSubsystem(
    RobotConstants.kShooterMotorPort, 
    RobotConstants.kRotationMotorPort);
  private final ClimbSubsystem m_climb = new ClimbSubsystem(
    RobotConstants.kClimbMotorPort);
  //creating the controllers, allows our controllers to be detected by our programming
  Joystick m_driverController = new Joystick(RobotConstants.kDriverControllerPort); //kDriverControllerPort is the port on which the driver controller is connected to
  Joystick m_manipulatorController = new Joystick(RobotConstants.kManipulatorControllerPort); //kManipulatorControllerPort is the port on which the manipulator controller is connected to

  //initializes dropdown menu for autos, will be sent to Shuffleboard/Smart Dashboard
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    //fetches the "tx" value from the front limelight
    double tx = LimelightHelpers.getTX("limelight-front");
    //double yaw = LimelightHelpers.getTV("limelight-front") ? LimelightHelpers.getTargetPose_RobotSpace("front")[4] : 0;

    LimelightHelpers.setLEDMode_ForceOff("limelight-front");

    //registers commands to be used in PathPlanner, allowing us to use these commands during auto
    NamedCommands.registerCommand("Reset Gyro", new InstantCommand(() -> m_drive.resetGyro(), m_drive));
    NamedCommands.registerCommand("Rumble On", new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 1)));
    NamedCommands.registerCommand("Rumble Off", new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 0)));

    //binds controller buttons to commands
    configureBindings();

    //creates dropdown to select autos
    autoChooser = AutoBuilder.buildAutoChooser();

    //default command for drive subsystem
    m_drive.setDefaultCommand(
      new RunCommand( 
        () -> m_drive.drive(
          squared(-MathUtil.applyDeadband(m_driverController.getRawAxis(RobotConstants.kLeftYAxisPort), .05)), 
          squared(-MathUtil.applyDeadband(m_driverController.getRawAxis(RobotConstants.kLeftXAxisPort), .05)),
          squared(-MathUtil.applyDeadband(m_driverController.getRawAxis(RobotConstants.kRightXAxisPort), .05)), 
          true), //turn to false to get rid of field relative
      m_drive));

            //squared(-MathUtil.applyDeadband(m_driverController.getRawAxis(OperatorConstants.kRightXAxisPort),.05))

    //Publishes data on SmartDashboard/Shuffleboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData(m_drive.m_gyro);
    SmartDashboard.putData(m_drive.m_field);
    SmartDashboard.putNumber("x", tx);
  }

  /**
   * Squares a value while keeping the sign of the original number. This is used for smoother and more precise driving.
   * 
   * @param value Value to be squared
   * @return Squared value
   */
  public double squared(double value) {
    return value >= 0 ? value*value : -(value*value);
  }

  /**
   * Configures button bindings.
   */
  private void configureBindings() {
    //initializing driver buttons
    Trigger driverBButton = new JoystickButton(
      m_driverController, 
      RobotConstants.kDriverBButton);
    Trigger driverAButton = new JoystickButton(
      m_driverController, 
      RobotConstants.kDriverAButton);
    Trigger driverXButton = new JoystickButton(
      m_driverController, 
      RobotConstants.kDriverXButton);
    Trigger driverYButton = new JoystickButton(
      m_driverController, 
      RobotConstants.kDriverYButton);
    Trigger driverLeftShoulder = new JoystickButton(
      m_driverController, 
      RobotConstants.kDriverLeftShoulder);
    Trigger driverRightShoulder = new JoystickButton(
      m_driverController, 
      RobotConstants.kDriverRightShoulder);
    Trigger driverPlusButton = new JoystickButton(
      m_driverController, 
      RobotConstants.kDriverPlusButton);
    Trigger driverMinusButton = new JoystickButton(
      m_driverController,
       RobotConstants.kDriverMinusButton);
    Trigger driverDPadUp = new POVButton(
      m_driverController, 
      RobotConstants.kPOVUp);
    Trigger driverDPadRight = new POVButton(
      m_driverController, 
      RobotConstants.kPOVRight);
    Trigger driverDPadDown = new POVButton(
      m_driverController, 
      RobotConstants.kPOVDown);
    Trigger driverDPadLeft = new POVButton(
      m_driverController, 
      RobotConstants.kPOVLeft);
    Trigger driverLeftTrigger = new AnalogTrigger(
      m_driverController, 
      RobotConstants.kDriverLeftTriggerAxis, 
      0.5);
    Trigger driverRightTrigger = new AnalogTrigger(
      m_driverController, 
      RobotConstants.kDriverRightTriggerAxis, 
      0.5);

    //initiating manipulator buttons
    Trigger manipulatorBButton = new JoystickButton(
      m_manipulatorController, 
      RobotConstants.kManipulatorBButton);
    Trigger manipulatorAButton = new JoystickButton(
      m_manipulatorController, 
      RobotConstants.kManipulatorAButton);
    Trigger manipulatorXButton = new JoystickButton(
      m_manipulatorController, 
      RobotConstants.kManipulatorXButton);
    Trigger manipulatorYButton = new JoystickButton(
      m_manipulatorController, 
      RobotConstants.kManipulatorYButton);
    Trigger manipulatorLeftShoulder = new JoystickButton(
      m_manipulatorController, 
      RobotConstants.kManipulatorLeftShoulder);
    Trigger manipulatorRightShoulder = new JoystickButton(
      m_manipulatorController, 
      RobotConstants.kManipulatorRightShoulder);
    Trigger manipulatorPlusButton = new JoystickButton(
      m_manipulatorController, 
      RobotConstants.kManipulatorPlusButton);
    Trigger manipulatorMinusButton = new JoystickButton(
      m_manipulatorController,
       RobotConstants.kManipulatorMinusButton);
    Trigger manipulatorLeftJoystick = new JoystickButton(
      m_manipulatorController, 
      RobotConstants.kManipulatorPlusButton);
    Trigger manipulatorRightJoystick = new JoystickButton(
      m_manipulatorController,
       RobotConstants.kManipulatorMinusButton);
    Trigger manipulatorDPadUp = new POVButton(
      m_manipulatorController, 
      RobotConstants.kPOVUp);
    Trigger manipulatorDPadRight = new POVButton(
      m_manipulatorController, 
      RobotConstants.kPOVRight);
    Trigger manipulatorDPadDown = new POVButton(
      m_manipulatorController, 
      RobotConstants.kPOVDown);
    Trigger manipulatorDPadLeft = new POVButton(
      m_manipulatorController, 
      RobotConstants.kPOVLeft);
    Trigger manipulatorLeftTrigger = new AnalogTrigger(
      m_manipulatorController, 
      RobotConstants.kManipulatorLeftTriggerAxis, 
      0.5);
    Trigger manipulatorRightTrigger = new AnalogTrigger(
      m_manipulatorController, 
      RobotConstants.kManipulatorRightTriggerAxis, 
      0.5);

    //binding buttons to controls  
    driverBButton.onTrue(m_drive.resetGyro()); //reset gyro button
    driverXButton.whileTrue(new RunCommand(() -> m_drive.setX(), m_drive)); //handbrake button

    driverAButton.whileTrue(
      new RunCommand(
          () -> {
            if (LimelightHelpers.getTargetPose_RobotSpace("limelight-front").length != 0) {
              m_drive.drive(
              squared(-MathUtil.applyDeadband(m_driverController.getRawAxis(RobotConstants.kLeftYAxisPort), .05)),
              MathUtil.applyDeadband(MathUtil.clamp(-LimelightHelpers.getTargetPose_RobotSpace("limelight-front")[0], -0.25, 0.25), 0.01),
              MathUtil.applyDeadband(MathUtil.clamp(-LimelightHelpers.getTargetPose_RobotSpace("limelight-front")[4] * 0.01, -0.4, 0.4),0.03), 
              false);
            }
          }, 
          m_drive));

    driverLeftTrigger.whileTrue(new InstantCommand(() -> m_shooter.shoot(-0.3)));
    driverLeftTrigger.onFalse(new InstantCommand(() -> m_shooter.shoot(0)));
    driverRightTrigger.whileTrue(new InstantCommand(() -> m_shooter.shoot(0.3)));
    driverRightTrigger.onFalse(new InstantCommand(() -> m_shooter.shoot(0)));

    driverLeftShoulder.whileTrue(new InstantCommand(() -> m_shooter.shoot(-1)));
    driverLeftShoulder.onFalse(new InstantCommand(() -> m_shooter.shoot(0)));
    driverRightShoulder.whileTrue(new InstantCommand(() -> m_shooter.shoot(1)));
    driverRightShoulder.onFalse(new InstantCommand(() -> m_shooter.shoot(0)));
    
    manipulatorRightTrigger.whileTrue(new RunCommand(() -> m_elevator.ascend(0.3)));
    manipulatorRightTrigger.onFalse(new InstantCommand(() -> m_elevator.ascend(0)));
    manipulatorLeftTrigger.whileTrue(new RunCommand(() -> m_elevator.descend(0.3)));
    manipulatorLeftTrigger.onFalse(new InstantCommand(() -> m_elevator.descend(0)));

    manipulatorDPadUp.whileTrue(new RunCommand(() -> m_shooter.rotate(-0.1)));
    manipulatorDPadUp.onFalse(new InstantCommand(() -> m_shooter.rotate(0)));
    manipulatorDPadDown.whileTrue(new RunCommand(() -> m_shooter.rotate(0.1)));
    manipulatorDPadDown.onFalse(new InstantCommand(() -> m_shooter.rotate(0)));
    
    manipulatorDPadLeft.whileTrue(new RunCommand(() -> m_climb.rotate(0.3)));
    manipulatorDPadLeft.onFalse(new InstantCommand(() -> m_climb.rotate(0)));
    manipulatorDPadRight.whileTrue(new RunCommand(() -> m_climb.rotate(-0.3)));
    manipulatorDPadRight.onFalse(new InstantCommand(() -> m_climb.rotate(0)));
  

    //test


    

  }

  /**
   * Gets the elevator subsystem.
   * 
   * @return Elevator subsystem
   */
  public ElevatorSubsystem getElevator() {
    return m_elevator;
  }

  /**
   * Gets the shooter subsystem.
   * 
   * @return Shooter subsystem
   */
  public ShooterSubsystem getShooter() {
    return m_shooter;
  }

  /**
   * Gets the climb subsystem.
   * 
   * @return Climb subsystem
   */
  public ClimbSubsystem getClimb() {
    return m_climb;
  }

  /**
   * Gets the selected autonomous command from SmartDashboard.
   * 
   * @return The autonomous command to be run.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
