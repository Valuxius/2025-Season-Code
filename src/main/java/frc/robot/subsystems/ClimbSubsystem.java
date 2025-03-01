// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  //initializing the motor
  private final SparkMax m_climbMotor;

  //initializing the encoder
  private final RelativeEncoder m_climbEncoder;

  //initalizing the PID controller (disabled for now)
  private final SparkClosedLoopController m_climbPID;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem(int p_climbID) {
    //creating the motors
    m_climbMotor = new SparkMax(p_climbID, MotorType.kBrushless);

    //creating the encoders
    m_climbEncoder = m_climbMotor.getEncoder();

    //creating the PID controllers (disabled for now)
    m_climbPID = m_climbMotor.getClosedLoopController();
  }

  /**
   * Sets the climb to a certain angle.
   * 
   * @param angle Angle to be set.
   */
  public void setRotation(double angle) {
    m_climbMotor.set((angle - m_climbEncoder.getPosition()) * 0.01);
  }

  /**
   * Rotates the climb.
   * 
   * @param speed Speed to rotate at, 0 - 1.
   */
  public void rotate(double speed) {
    m_climbMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
