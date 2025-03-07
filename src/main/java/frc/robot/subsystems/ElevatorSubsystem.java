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

  //variables for the presets
  private int stage = 0;
  private boolean stageLock = false; //will toggle to false when needed, locks the elevator at preset if set to true

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

    //configures the motors with PID controls
    m_lMotor.configure(MotorConfigs.m_elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rMotor.configure(MotorConfigs.m_elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Moves the elevator up.
   * 
   * @param speed Speed of ascent, 0 - 1
   */
  public void ascend(double speed) {
    stageLock = false; //set to false to allow free moving
    double newSpeed = speed; //creates new variable for adjusted speed

    //reduces speed near the top as a "stop"
    if (m_lEncoder.getPosition() > 40) { //starts reducing speed after encoder position reaches 40
      newSpeed = Math.abs( //absolute value
        (((speed)/5)*(m_lEncoder.getPosition() - 45)) //linearly decreases speed to 0 at position 45
      ); 
    }

    //sets the motor speeds
    m_lMotor.set(newSpeed);
    m_rMotor.set(-newSpeed);

    //posts the data to SmartDashboard (for troubleshooting)
    SmartDashboard.putNumber("Speed", newSpeed);
  }

  /**
   * Moves the elevator down.
   * 
   * @param speed Speed of descent, 0 - 1
   */
  public void descend(double speed) {
    stageLock = false; //set to false to allow free moving
    double newSpeed = speed; //creates new variable for adjusted speed

    //reduces speed near the bottom as a "stop"
    if (m_lEncoder.getPosition() < 1) { //minimum speed at position 1
      newSpeed = 0.1;
    } else if (m_lEncoder.getPosition() < 5) { //starts reducing speed at position 5
      newSpeed = 0.1 + //minimum speed
      (((speed-0.1)/4)*(m_lEncoder.getPosition() - 1)); //linearly decreases speed to 0.1 at position 1 (5 - 4)
    }

    //sets motor speeds
    m_lMotor.set(-newSpeed);
    m_rMotor.set(newSpeed);

    //puts data on SmartDashboard for troubleshoooting
    SmartDashboard.putNumber("Speed", newSpeed);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    //sets the PID references to the encoder position
    m_lPID.setReference(m_lEncoder.getPosition(), ControlType.kPosition);
    m_rPID.setReference(m_rEncoder.getPosition(), ControlType.kPosition);
    
    //puts encoder position on SmartDashboard for troubleshooting
    SmartDashboard.putNumber("Elevator Encoder", m_lEncoder.getPosition());
    if (stageLock) {

    }
  }
}
