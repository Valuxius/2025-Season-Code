// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class RobotContainer {
  //creating the subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final LimelightSubsystem m_frontLimeLight = new LimelightSubsystem("limelight-front");

  //creating the controllers
  Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  Joystick m_manipulatorController = new Joystick(ManipulatorConstants.kManipulatorControllerPort);

  //dropdown menu for autos
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    //registers commands to use in path planner
    //NamedCommands.registerCommand("name", new Command(subsystem));

    //binds controller buttons to commands
    configureBindings();

    //creates dropdown to select autos
    autoChooser = AutoBuilder.buildAutoChooser();

    //default command for drive subsystem
    m_drive.setDefaultCommand(
      new DriveWithJoystick(
        m_drive, 
        () -> squared(m_driverController.getRawAxis(OperatorConstants.kLeftYAxisPort)),
        () -> squared(m_driverController.getRawAxis(OperatorConstants.kLeftXAxisPort)),
        () -> squared(m_driverController.getRawAxis(OperatorConstants.kRightXAxisPort))));

    //configures limelight pipeline at start (0 is apriltag pipeline)
    m_frontLimeLight.setPipeline(0);

    //Publishes data on SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData(m_drive.m_gyro);
    SmartDashboard.putData(m_drive.m_field);
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
    //gyro reset button
    Trigger driverBButton = new JoystickButton(m_driverController, OperatorConstants.kDriverBButton);
    driverBButton.onTrue(m_drive.resetGyro());
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
