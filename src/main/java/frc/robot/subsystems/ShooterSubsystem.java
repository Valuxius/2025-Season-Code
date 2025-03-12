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

public class ShooterSubsystem extends SubsystemBase {
  //initializing the motors
  private final SparkMax m_shooterMotor;
  private final SparkMax m_rotationMotor;

  //initializing the encoders
  private final RelativeEncoder m_shooterEncoder;
  private final RelativeEncoder m_rotationEncoder;

  //initalizing the PID controllers 
  private final SparkClosedLoopController m_shooterPID; //disabled for now
  private final SparkClosedLoopController m_rotationPID;

  //variable for preset
  private double preset = 0;

  //angle variables (0 should be straight up, increased values bring the shooter down)
  private double netPreset = 3;
  private double algaePreset = 3.5;
  private double floorPreset = 8;
  private double coralPreset = 11;
  private double humanPlayerPreset = 2;
  private double processorPreset = 8;

  //speed variable
  private double speed = 0.1;

  //rotation variable
  private double rotation = 0;

  //lock variable
  private boolean lock = false; 

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
    lock = false;
    m_rotationMotor.set(speed);
  }

  /**
   * Sets the preset of the shooter angle.
   * 
   * @param preset Preset number
   */
  public void setPreset(int preset) {
    lock = true;
    this.preset = preset;
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
    if (lock) {
      if (preset == 0) rotation = 0;
      else if (preset == 1) rotation = netPreset;
      else if (preset == 2) rotation = algaePreset;
      else if (preset == 3) rotation = floorPreset;
      else if (preset == 4) rotation = coralPreset;
      else if (preset == 5) rotation = humanPlayerPreset;
      else if (preset == 6) rotation = processorPreset;
      /* 
      if (m_rotationEncoder.getPosition() < rotation) {
        if (m_rotationEncoder.getPosition() > rotation - 2) {
          m_rotationMotor.set(MathUtil.applyDeadband(speed * ((rotation - m_rotationEncoder.getPosition())/2), 0.05));
        }
        else {
          m_rotationMotor.set(speed);
        }
      }
      else if (m_rotationEncoder.getPosition() > rotation) {
        if (m_rotationEncoder.getPosition() < rotation + 2) {
          m_rotationMotor.set(-MathUtil.applyDeadband(speed * ((m_rotationEncoder.getPosition() - rotation)/2), 0.05));
        }
        else {
          m_rotationMotor.set(-speed);
        }
      }*/
      m_rotationPID.setReference(rotation, ControlType.kPosition);
    }

    else {
      m_rotationPID.setReference(m_rotationEncoder.getPosition(), ControlType.kPosition);
    }
    
    SmartDashboard.putNumber("Shooter Encoder", m_shooterEncoder.getPosition());
  }
}
