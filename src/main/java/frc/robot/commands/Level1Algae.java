// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Level1Algae extends Command {
  private final ElevatorSubsystem m_elevator;
  private final ShooterSubsystem m_shooter;
  /** Creates a new Level1Coral. */
  public Level1Algae(ElevatorSubsystem m_elevator, ShooterSubsystem m_shooter) {
    this.m_elevator = m_elevator;
    this.m_shooter = m_shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_elevator, this.m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setPreset(5);
    m_shooter.setPreset(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
