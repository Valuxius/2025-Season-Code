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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorConfigs;

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

  private int stage = 0;
  private boolean stageLock = false;

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

    //resets the encoders
    m_lEncoder.setPosition(0);
    m_rEncoder.setPosition(0);

    m_lMotor.configure(MotorConfigs.m_elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rMotor.configure(MotorConfigs.m_elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Moves the elevator up.
   * 
   * @param speed Speed of ascent, 0 - 1
   */
  public void ascend(double speed) {
    stageLock = false;
    double newSpeed = speed;
    if (m_lEncoder.getPosition() > 40) {
      newSpeed = Math.abs((((speed)/5)*(m_lEncoder.getPosition() - 45)));
    }
    m_lMotor.set(speed);
    m_rMotor.set(-speed);
    SmartDashboard.putNumber("Speed", newSpeed);
  }

  /**
   * Moves the elevator down.
   * 
   * @param speed Speed of descent, 0 - 1
   */
  public void descend(double speed) {
    stageLock = false;
    double newSpeed = speed;
    if (m_lEncoder.getPosition() < 1) {
      newSpeed = 0.1;
    } else if (m_lEncoder.getPosition() < 5) {
      newSpeed = 0.1 + (((speed-0.1)/4)*(m_lEncoder.getPosition() - 1));
    }
    m_lMotor.set(-newSpeed);
    m_rMotor.set(newSpeed);
    SmartDashboard.putNumber("Speed", newSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_lPID.setReference(m_lEncoder.getPosition(), ControlType.kPosition);
    m_rPID.setReference(m_rEncoder.getPosition(), ControlType.kPosition);
    SmartDashboard.putNumber("Elevator Encoder", m_lEncoder.getPosition());
    if (stageLock) {
      
    }
  }
}
