// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveWithJoystick extends Command {
  private final Supplier<Double> xSpeed;
  private final Supplier<Double> ySpeed;
  private final Supplier<Double> rot;
  private final DriveSubsystem m_drive;
  /** Creates a new DriveWithJoystick. */
  public DriveWithJoystick(DriveSubsystem m_drive, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rot) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rot = rot;
    this.m_drive = m_drive;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.drive(0, 0, 0, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(
      -MathUtil.applyDeadband(xSpeed.get(), 0.08), 
      -MathUtil.applyDeadband(ySpeed.get(), 0.08), 
      -MathUtil.applyDeadband(rot.get(), 0.08), 
      true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
