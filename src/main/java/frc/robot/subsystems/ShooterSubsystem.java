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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorConfigs;

public class ShooterSubsystem extends SubsystemBase {
  //initializing the motors
  private final SparkMax m_shooterMotor;
  private final SparkMax m_rotationMotor;

  //initializing the encoders
  private final RelativeEncoder m_shooterEncoder;
  private final RelativeEncoder m_rotationEncoder;

  //initalizing the PID controllers 
  private final SparkClosedLoopController m_shooterPID;
  private final SparkClosedLoopController m_rotationPID;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(int p_shooterID, int p_rotationID) {
    //creating the motors
    m_shooterMotor = new SparkMax(p_shooterID, MotorType.kBrushless);
    m_rotationMotor = new SparkMax(p_rotationID, MotorType.kBrushless);

    //creating the encoders
    m_shooterEncoder = m_shooterMotor.getEncoder();
    m_rotationEncoder = m_rotationMotor.getEncoder();

    //creating the PID controllers
    m_shooterPID = m_shooterMotor.getClosedLoopController(); //disabled
    m_rotationPID = m_rotationMotor.getClosedLoopController();

    //resets the encoder
    m_shooterEncoder.setPosition(0);
    m_rotationEncoder.setPosition(0);

    //configures the PIDs for the motors
    m_shooterMotor.configure(MotorConfigs.m_shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rotationMotor.configure(MotorConfigs.m_shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Runs the shooter to output the game piece.
   * 
   * @param speed Speed to shoot at, 0 - 1.
   */
  public void shoot(double speed) {
    m_shooterMotor.set(speed);
  }

  /**
   * Sets the shooter at a certain angle.
   * 
   * @param angle Angle to be set.
   */
  public void setRotation(double angle) {
    m_rotationMotor.set((angle - m_rotationEncoder.getPosition()) * 0.01);
  }

  /**
   * Rotates the shooter.
   * 
   * @param speed Speed to rotate at, 0 - 1.
   */
  public void rotate(double speed) {
    m_rotationMotor.set(speed);
  }

  /**
   * Gets the shooter's rotation PID controller.
   * 
   * @return Rotation PID controler
   */
  public SparkClosedLoopController getRotationPID() {
    return m_rotationPID;
  }

  /**
   * Gets the shooter's rotation encoder.
   * 
   * @return Shooter rotation encoder
   */
  public RelativeEncoder getRotationEncoder() {
    return m_rotationEncoder;
  }

  

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    //sets the encoder reference for the PID
    m_rotationPID.setReference(m_rotationEncoder.getPosition(), ControlType.kPosition);
  }
}
