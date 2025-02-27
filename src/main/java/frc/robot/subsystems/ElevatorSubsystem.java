// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  //initializing the motors
  private final SparkMax m_lMotor;
  private final SparkMax m_rMotor;

  //initializing the encoders
  private final RelativeEncoder m_lEncoder;
  private final RelativeEncoder m_rEncoder;

  //initializing the PID controllers (disabled for now)
  private final SparkClosedLoopController m_lPID;
  private final SparkClosedLoopController m_rPID;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(int p_leftID, int p_rightID) {
    //creating the motors for the subsystem
    m_lMotor = new SparkMax(p_leftID, MotorType.kBrushless);
    m_rMotor = new SparkMax(p_rightID, MotorType.kBrushless);

    //creating the encoders for the subsystem
    m_lEncoder = m_lMotor.getEncoder();
    m_rEncoder = m_rMotor.getEncoder();

    //creating the PID controllers (disabled for now)
    m_lPID = m_lMotor.getClosedLoopController();
    m_rPID = m_rMotor.getClosedLoopController();
  }

  /**
   * Moves the elevator up.
   * 
   * @param speed Speed of ascent, 0 - 1
   */
  public void ascend(double speed) {
    m_lMotor.set(speed);
    m_rMotor.set(-speed);
  }

  /**
   * Moves the elevator down.
   * 
   * @param speed Speed of descent, 0 - 1
   */
  public void descend(double speed) {
    m_lMotor.set(-speed);
    m_rMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
