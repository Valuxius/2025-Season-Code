// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.utils.AdjustedGyro;
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
  private final LEDSubsystem m_led = new LEDSubsystem();

  //creating the controllers, allows our controllers to be detected by our programming
  Joystick m_driverController = new Joystick(RobotConstants.kDriverControllerPort); //kDriverControllerPort is the port on which the driver controller is connected to
  Joystick m_manipulatorController = new Joystick(RobotConstants.kManipulatorControllerPort); //kManipulatorControllerPort is the port on which the manipulator controller is connected to

  //initializes dropdown menu for autos, will be sent to Shuffleboard/Smart Dashboard
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    //makes sure our limelight LEDs are off
    LimelightHelpers.setLEDMode_ForceOff("limelight-front");

    //registers commands to be used in PathPlanner, allowing us to use these commands during auto
    NamedCommands.registerCommand("Reset Gyro", new InstantCommand(() -> m_drive.resetGyro(), m_drive));
    NamedCommands.registerCommand("Rumble On", new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 1)));
    NamedCommands.registerCommand("Rumble Off", new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 0)));
    NamedCommands.registerCommand("Level 2 Elevator", new InstantCommand(() -> {m_elevator.setPreset(2); m_shooter.setPreset(5);}));
    NamedCommands.registerCommand("Level 3 Elevator", new InstantCommand(() -> {m_elevator.setPreset(3); m_shooter.setPreset(5);}));
    NamedCommands.registerCommand("Level 1 Algae", new InstantCommand(() -> {m_elevator.setPreset(5); m_shooter.setPreset(2);}));
    NamedCommands.registerCommand("Stop Shooter", new InstantCommand(() -> m_shooter.shoot(0)));
    NamedCommands.registerCommand("Slow Shoot", new InstantCommand(() -> m_shooter.shoot(0.3)));
    NamedCommands.registerCommand("Fast Intake", new InstantCommand(() -> m_shooter.shoot(-1)));

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

    //Publishes data on SmartDashboard/Shuffleboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
      RobotConstants.kManipulatorLeftJoystick);
    Trigger manipulatorRightJoystick = new JoystickButton(
      m_manipulatorController,
       RobotConstants.kManipulatorRightJoystick);
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

    //detection for when robot is stationary/not being controlled
    Trigger gyroStop = new Trigger(
      () -> (Math.abs(m_driverController.getRawAxis(RobotConstants.kLeftYAxisPort)) <= 0.05 &&
             Math.abs(m_driverController.getRawAxis(RobotConstants.kLeftYAxisPort)) <= 0.05 &&
             Math.abs(m_driverController.getRawAxis(RobotConstants.kRightXAxisPort)) <= 0.05)
    );

    Trigger lightChange = new Trigger(() -> m_elevator.isMoving() || m_shooter.isMoving());

    //stops gyro when robot is stationary, starts gyro again when robot is moving
    gyroStop.onTrue(new InstantCommand(() -> AdjustedGyro.setGyroStop(true)));
    gyroStop.whileFalse(new RunCommand(() -> AdjustedGyro.setGyroStop(false)));

    lightChange.whileTrue(new RunCommand(() -> m_led.setPattern(LEDPattern.solid(Color.kGreen)), m_led));
    lightChange.onTrue(new InstantCommand(() -> m_led.setMoving(true)));
    lightChange.onFalse(new InstantCommand(() -> m_led.setMoving(false)));

    //binding buttons to controls  
    driverBButton.onTrue(m_drive.resetGyro()); //reset gyro button
    driverXButton.whileTrue(new RunCommand(() -> m_drive.setX(), m_drive)); //handbrake button

    //left reef alignment command
    driverDPadLeft.whileTrue(
      new RunCommand(
          () -> {
            //makes sure 3d localization is on before running, preventing code crash
            if (LimelightHelpers.getTargetPose_RobotSpace("limelight-front").length != 0) {
              m_drive.drive(
                //raw controller input for forward and backward
                squared(-MathUtil.applyDeadband(m_driverController.getRawAxis(RobotConstants.kLeftYAxisPort), .05)),
                //aligns on the horizontal axis with the april tag, then offsets for left side of the reef
                (LimelightHelpers.getTV("limelight-front")) ? 
                  MathUtil.applyDeadband(MathUtil.clamp(
                    -LimelightHelpers.getTargetPose_RobotSpace("limelight-front")[0]+0.125, -0.25, 0.25), 0.075) 
                  : 0,
                //rotates the robot to make it parallel to the april tag
                MathUtil.applyDeadband(MathUtil.clamp(-LimelightHelpers.getTargetPose_RobotSpace("limelight-front")[4] * 0.01, -0.4, 0.4),0.03), 
                false
              );
            }
          }, 
        m_drive));

    //right reef alignment command
    driverDPadRight.whileTrue(
      new RunCommand(
          () -> {
            //makes sure 3d localization is on before running, preventing code crash
            if (LimelightHelpers.getTargetPose_RobotSpace("limelight-front").length != 0) {
              m_drive.drive(
                //raw controller input for forward and backward
                squared(-MathUtil.applyDeadband(m_driverController.getRawAxis(RobotConstants.kLeftYAxisPort), .05)),
                //aligns on the horizontal axis with the april tag, then offsets for right side of the reef
                (LimelightHelpers.getTV("limelight-front")) ? 
                  MathUtil.applyDeadband(MathUtil.clamp( 
                    -LimelightHelpers.getTargetPose_RobotSpace("limelight-front")[0]-0.2, -0.25, 0.25), 0.01) 
                  : 0,
                //rotates the robot to make it parallel to the april tag
                MathUtil.applyDeadband(MathUtil.clamp(-LimelightHelpers.getTargetPose_RobotSpace("limelight-front")[4] * 0.01, -0.4, 0.4),0.03), 
                false
              );
            }
          }, 
        m_drive));

    //center reef alignment 
    driverDPadUp.whileTrue(
      new RunCommand(
          () -> {
            //makes sure 3d localization is on before running, preventing code crash
            if (LimelightHelpers.getTargetPose_RobotSpace("limelight-front").length != 0) {
              m_drive.drive(
                //raw controller input for forward and backward
                squared(-MathUtil.applyDeadband(m_driverController.getRawAxis(RobotConstants.kLeftYAxisPort), .05)),
                //aligns on the horizontal axis with the april tag, then offsets for center of the reef
                (LimelightHelpers.getTV("limelight-front")) ? 
                  MathUtil.applyDeadband(MathUtil.clamp(
                    -LimelightHelpers.getTargetPose_RobotSpace("limelight-front")[0]-0.05, -0.25, 0.25), 0.01) 
                  : 0,
                //rotates the robot to make it parallel to the april tag
                MathUtil.applyDeadband(MathUtil.clamp(-LimelightHelpers.getTargetPose_RobotSpace("limelight-front")[4] * 0.01, -0.4, 0.4),0.03), 
                false
              );
            }
          }, 
        m_drive));

    //manual elevator control
    manipulatorDPadLeft.whileTrue(new RunCommand(() -> m_elevator.ascend(0.3), m_elevator)); //ascend
    manipulatorDPadLeft.onFalse(new InstantCommand(() -> m_elevator.ascend(0)));
    manipulatorDPadRight.whileTrue(new RunCommand(() -> m_elevator.descend(0.3), m_elevator)); //descend
    manipulatorDPadRight.onFalse(new InstantCommand(() -> m_elevator.descend(0)));

    //manual shooter control
    manipulatorPlusButton.whileTrue(new RunCommand(() -> m_shooter.rotate(0.15), m_shooter)); //tilt up
    manipulatorPlusButton.onFalse(new InstantCommand(() -> m_shooter.rotate(0)));
    manipulatorMinusButton.whileTrue(new RunCommand(() -> m_shooter.rotate(-0.15), m_shooter)); //tilt down
    manipulatorMinusButton.onFalse(new InstantCommand(() -> m_shooter.rotate(0)));

    //manual climb control
    driverRightShoulder.whileTrue(new RunCommand(() -> m_climb.rotate(0.48), m_climb)); //tilt up
    driverRightShoulder.onFalse(new InstantCommand(() -> m_climb.rotate(0)));
    driverLeftShoulder.whileTrue(new RunCommand(() -> m_climb.rotate(-0.3), m_climb)); //tilt down
    driverLeftShoulder.onFalse(new InstantCommand(() -> m_climb.rotate(0)));
    
    //slow intakes and outtakes
    manipulatorLeftTrigger.whileTrue(new RunCommand(() -> m_shooter.shoot(-0.3))); //slow intake
    manipulatorLeftTrigger.onFalse(new InstantCommand(() -> m_shooter.shoot(0)));
    manipulatorRightTrigger.whileTrue(new RunCommand(() -> m_shooter.shoot(0.3))); //slow outtake
    manipulatorRightTrigger.onFalse(new InstantCommand(() -> m_shooter.shoot(0)));

    //fast intakes and outtakes
    manipulatorLeftShoulder.whileTrue(new InstantCommand(() -> m_shooter.shoot(-1))); //fast intake
    manipulatorLeftShoulder.onFalse(new InstantCommand(() -> m_shooter.shoot(0)));
    manipulatorRightShoulder.whileTrue(new InstantCommand(() -> m_shooter.shoot(1))); //fast outtake
    manipulatorRightShoulder.onFalse(new InstantCommand(() -> m_shooter.shoot(0)));

    //algae presets
    manipulatorRightJoystick.onTrue(new InstantCommand(() -> {m_elevator.setPreset(5); m_shooter.setPreset(2);})); //lower algae
    manipulatorLeftJoystick.onTrue(new InstantCommand(() -> {m_elevator.setPreset(6); m_shooter.setPreset(2);})); //higher algae
    
    //net preset
    manipulatorDPadUp.onTrue(new InstantCommand(() -> {m_elevator.setPreset(9); m_shooter.setPreset(1);}));

    //processor preset
    manipulatorDPadDown.onTrue(new InstantCommand(() -> {m_elevator.setPreset(7); m_shooter.setPreset(8);}));
    
    //reset button
    manipulatorXButton.onTrue(new InstantCommand(() -> {m_elevator.setPreset(0); m_shooter.setPreset(0);}));

    //coral presets
    manipulatorYButton.onTrue(new InstantCommand(() -> {m_elevator.setPreset(1); m_shooter.setPreset(4);})); //L1
    manipulatorBButton.onTrue(new InstantCommand(() -> {m_elevator.setPreset(2); m_shooter.setPreset(5);})); //L2
    manipulatorAButton.onTrue(new InstantCommand(() -> {m_elevator.setPreset(3); m_shooter.setPreset(6);})); //L3
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
