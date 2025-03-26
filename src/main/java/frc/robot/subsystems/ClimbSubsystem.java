// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorConfigs;

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

    m_climbEncoder.setPosition(0);

    m_climbMotor.configure(MotorConfigs.m_climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    double newSpeed = speed;
    if (m_climbEncoder.getPosition() <= 0 && speed < 0) {
      newSpeed = 0;
    }
    else if (m_climbEncoder.getPosition() > 180 && speed > 0) {
      newSpeed = 0;
    }
    m_climbMotor.set(newSpeed);
  }

  /**
   * Gets the climb PID controller.
   * 
   * @return Climb PID controller
   */
  public SparkClosedLoopController getClimbPID() {
    return m_climbPID;
  }

  /**
   * Gets the climb encoder.
   * 
   * @return Climb encoder
   */
  public RelativeEncoder getClimbEncoder() {
    return m_climbEncoder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_climbPID.setReference(m_climbEncoder.getPosition(), ControlType.kPosition);
    SmartDashboard.putNumber("ClimbEncoder", m_climbEncoder.getPosition());
  }
}
