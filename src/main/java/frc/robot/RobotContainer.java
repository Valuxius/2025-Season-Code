// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.utils.LimelightHelpers;

public class RobotContainer {
  //creating the subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  /*private final ElevatorSubsystem m_elevator = new ElevatorSubsystem(
    DriveConstants.kLeftElevatorMotorPort, 
    DriveConstants.kRightElevatorMotorPort);
  */

  //creating the controllers, allows our controllers to be detected by our programming
  Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort); //kDriverControllerPort is the port on which the driver controller is connected to
  Joystick m_manipulatorController = new Joystick(ManipulatorConstants.kManipulatorControllerPort); //kManipulatorControllerPort is the port on which the manipulator controller is connected to

  //initializes dropdown menu for autos, will be sent to Shuffleboard/Smart Dashboard
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    //fetches the "tx" value from the back limelight
    double tx = LimelightHelpers.getTX("limelight_back");

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
                squared(-MathUtil.applyDeadband(m_driverController.getRawAxis(OperatorConstants.kLeftYAxisPort), .05)), 
                squared(-MathUtil.applyDeadband(m_driverController.getRawAxis(OperatorConstants.kLeftXAxisPort), .05)),
                squared(-MathUtil.applyDeadband(m_driverController.getRawAxis(OperatorConstants.kRightXAxisPort),.05)), 
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
      OperatorConstants.kDriverBButton
    );
    Trigger driverAButton = new JoystickButton(
      m_driverController, 
      OperatorConstants.kDriverAButton
    );
    Trigger driverXButton = new JoystickButton(
      m_driverController, 
      OperatorConstants.kDriverXButton
    );
    Trigger driverYButton = new JoystickButton(
      m_driverController, 
      OperatorConstants.kDriverYButton
    );

    //initiating manipulator buttons
    Trigger manipulatorXButton = new JoystickButton(
      m_manipulatorController, 
      ManipulatorConstants.kManipulatorXButton
    );
    Trigger manipulatorYButton = new JoystickButton(
      m_manipulatorController, 
      ManipulatorConstants.kManipulatorYButton
    );

    //binding buttons to controls  
    driverBButton.onTrue(m_drive.resetGyro()); //reset gyro button
    driverAButton.whileTrue(new RunCommand(() -> m_drive.setX(), m_drive)); //handbrake button

    /*driverLeftTrigger.whileTrue(new RunCommand(
      () -> m_elevator.ascend(DriveConstants.kElevatorMaxSpeed))); //move elevator up
    driverRightTrigger.whileTrue(new RunCommand(
      () -> m_elevator.descend(DriveConstants.kElevatorMaxSpeed))); //move elevator down
       */

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
