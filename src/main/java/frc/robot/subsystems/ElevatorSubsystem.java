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
  private double stage = 0;
  private boolean stageLock = false; //will toggle to false when needed, locks the elevator at preset if set to true
  
  //heights for presets
  private static double stage1Height = 12.5;
  private static double stage2Height = 27;
  private static double stage3Height = 41;

  //speed for presets (0 - 1)
  private static double elevatorSpeed = 0.3;

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
    if (m_lEncoder.getPosition() > 42) { //starts reducing speed after encoder position reaches 42
      newSpeed = Math.abs( //absolute value
        (((speed)/3)*(m_lEncoder.getPosition() - 45)) //linearly decreases speed to 0 at position 45
      ); 
    } 

    //sets the motor speeds
    m_lMotor.set(MathUtil.applyDeadband(newSpeed, 0.1));
    m_rMotor.set(-MathUtil.applyDeadband(newSpeed, 0.1));

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
      newSpeed = 0.05;
    } else if (m_lEncoder.getPosition() < 5) { //starts reducing speed at position 5
      newSpeed = 0.05 + //minimum speed
      (((speed-0.05)/5)*(m_lEncoder.getPosition() - 1)); //linearly decreases speed to 0.1 at position 1 (5 - 4)
    }

    //sets motor speeds
    m_lMotor.set(-newSpeed);
    m_rMotor.set(newSpeed);

    //puts data on SmartDashboard for troubleshoooting
    SmartDashboard.putNumber("Speed", newSpeed);
  }

  /**
   * Increase the elevator stage.
   */
  public void increaseStage() {
    stageLock = false;
    stage++;
  }

  /**
   * Decrease the elevator stage.
   */
  public void decreaseStage() {
    stageLock = true;
    if (stage > 0) stage--;
  }

  /**
   * Sets the elevator stage.
   * 
   * @param stage Stage to be set to.
   */
  public void setStage(double stage) {
    stageLock = true;
    if (stage >= 0) this.stage = stage;
  }

  /**
   * Resets the elevator encoders.
   */
  public void resetEncoders() {
    m_lEncoder.setPosition(0);
    m_rEncoder.setPosition(0);
  }

  /**
   * Gets the left elevator PID controller.
   * 
   * @return Left elevator PID
   */
  public SparkClosedLoopController getLeftPID() {
    return m_lPID;
  }

  /**
   * Gets the right elevator PID controler.
   * 
   * @return Right elevator PID
   */
  public SparkClosedLoopController getRightPID() {
    return m_rPID;
  }

  /**
   * Gets the left elevator encoder. 
   * 
   * @return Elevator left encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return m_lEncoder;
  }

  /**
   * Gets the right elevator encoder.
   * 
   * @return Elevator right encoder
   */
  public RelativeEncoder getRightEncoder() {
    return m_lEncoder;
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    //sets the PID references to the encoder position
    m_lPID.setReference(m_lEncoder.getPosition(), ControlType.kPosition);
    m_rPID.setReference(m_rEncoder.getPosition(), ControlType.kPosition);
    SmartDashboard.putNumber("Stage", stage);
    
    //puts encoder position on SmartDashboard for troubleshooting
    SmartDashboard.putNumber("Elevator Encoder", m_lEncoder.getPosition());
    if (stageLock) {
      if (stage == 0) {
        if (m_lEncoder.getPosition() < 5) {
          m_lMotor.set(-0.1 * (m_lEncoder.getPosition()/5));
          m_rMotor.set(0.1 * (m_lEncoder.getPosition()/5));
        } 
        else if (m_lEncoder.getPosition() < 10) {
          m_lMotor.set(-(0.1 + (((0.2)/5)*(m_lEncoder.getPosition()))));
          m_rMotor.set(0.1 + ((0.2)/5)*(m_lEncoder.getPosition()));
        } 
        else {
          m_lMotor.set(-elevatorSpeed);
          m_rMotor.set(elevatorSpeed);
        }
        if (m_lEncoder.getVelocity() >= -0.01 && m_lEncoder.getPosition() < 3) {
          resetEncoders();
        }
      }
      
      else if (stage == 1) {
        if (m_lEncoder.getPosition() < stage1Height) {
          m_lMotor.set(MathUtil.applyDeadband(elevatorSpeed * ((stage1Height - m_lEncoder.getPosition())/stage1Height), 0.05));
          m_rMotor.set(-MathUtil.applyDeadband(elevatorSpeed * ((stage1Height - m_lEncoder.getPosition())/stage1Height), 0.05));
        } 
        else if (m_lEncoder.getPosition() > stage1Height) {
          if (m_lEncoder.getPosition() < stage1Height + 5) {
            m_lMotor.set(-MathUtil.applyDeadband(elevatorSpeed * ((m_lEncoder.getPosition() - stage1Height)/5), 0.05));
            m_rMotor.set(MathUtil.applyDeadband(elevatorSpeed * ((m_lEncoder.getPosition() - stage1Height)/5), 0.05));
          }
          else {
            m_lMotor.set(-elevatorSpeed);
            m_rMotor.set(elevatorSpeed);
          }
        }
      }

      else if (stage == 2) {
        if (m_lEncoder.getPosition() < stage2Height) {
          if (m_lEncoder.getPosition() > stage2Height - 5) {
            m_lMotor.set(MathUtil.applyDeadband(elevatorSpeed * ((stage2Height - m_lEncoder.getPosition())/5), 0.05));
            m_rMotor.set(-MathUtil.applyDeadband(elevatorSpeed * ((stage2Height - m_lEncoder.getPosition())/5), 0.05));
          }
          else {
            m_lMotor.set(elevatorSpeed);
            m_rMotor.set(-elevatorSpeed);
          }
        } 
        else if (m_lEncoder.getPosition() > stage2Height) {
          if (m_lEncoder.getPosition() < stage2Height + 5) {
            m_lMotor.set(-MathUtil.applyDeadband(elevatorSpeed * ((m_lEncoder.getPosition() - stage2Height)/5), 0.05));
            m_rMotor.set(MathUtil.applyDeadband(elevatorSpeed * ((m_lEncoder.getPosition() - stage2Height)/5), 0.1));
          }
          else {
            m_lMotor.set(-elevatorSpeed);
            m_rMotor.set(elevatorSpeed);
          }
        }
      }

      else if (stage == 3) {
        if (m_lEncoder.getPosition() < stage3Height) {
          if (m_lEncoder.getPosition() > stage3Height - 5) {
            m_lMotor.set(MathUtil.applyDeadband(elevatorSpeed * ((stage3Height - m_lEncoder.getPosition())/5), 0.05));
            m_rMotor.set(-MathUtil.applyDeadband(elevatorSpeed * ((stage3Height - m_lEncoder.getPosition())/5), 0.05));
          }
          else {
            m_lMotor.set(elevatorSpeed);
            m_rMotor.set(-elevatorSpeed);
          }
        } 
        else if (m_lEncoder.getPosition() > stage3Height) {
          if (m_lEncoder.getPosition() < stage3Height + 5) {
            m_lMotor.set(-MathUtil.applyDeadband(elevatorSpeed * ((m_lEncoder.getPosition() - stage3Height)/5), 0.05));
            m_rMotor.set(MathUtil.applyDeadband(elevatorSpeed * ((m_lEncoder.getPosition() - stage3Height)/5), 0.05));
          }
          else {
            m_lMotor.set(-elevatorSpeed);
            m_rMotor.set(elevatorSpeed);
          }
        }
      } 
    }
  }
}
