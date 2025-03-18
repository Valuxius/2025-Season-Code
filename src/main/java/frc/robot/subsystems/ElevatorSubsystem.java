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
import frc.robot.utils.TrapezoidalPositionControl;

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
  private double preset = 0;
  private boolean lock = true; //will toggle to false when needed, locks the elevator at preset if set to true
  
  //height instance variable
  private double height = 0;
  
  //heights for presets
  private double stage1Height = 12.5; //L1 Coral Height
  private double stage2Height = 27; //L2 Coral Height
  private double stage3Height = 41; //L3 Coral Height
  private double maxHeight = 43; //Max Height

  private double floorAlgaeHeight = 10; //Floor Algae Height
  private double l1AlgaeHeight = 0; //L1 Algae Height
  private double l2AlgaeHeight = 26.5; //L2 Algae Height

  private double processorHeight = 2; //Processor Height

  private double humanPlayerHeight = 14; //Human Player Height

  //speed for presets (0 - 1)
  private double elevatorSpeed = 0.6;

  private final TrapezoidalPositionControl m_heightProfile;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(int p_leftID, int p_rightID) {
    m_heightProfile = new TrapezoidalPositionControl(20, 1);

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
    lock = false; //set to false to allow free moving
    double newSpeed = speed; //creates new variable for adjusted speed

    //reduces speed near the top as a "stop"
    if (m_lEncoder.getPosition() > maxHeight - 3) { //starts reducing speed after encoder position reaches 3 below maxHeight
      newSpeed = Math.abs( //absolute value
        (((speed)/3)*(m_lEncoder.getPosition() - maxHeight)) //linearly decreases speed to 0 at maxHeight
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
    lock = false; //set to false to allow free moving
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
   * Sets the elevator stage.
   * 
   * @param stage Stage to be set to.
   */
  public void setPreset(double preset) {
    lock = true;
    if (preset >= 0) this.preset = preset;
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
    
    //puts encoder position on SmartDashboard for troubleshooting
    SmartDashboard.putNumber("Elevator Encoder", m_lEncoder.getPosition());
    if (lock) {
      //setting the height
      if (preset == 0) height = 0; //starting height
      else if (preset == 1) height = stage1Height; //L1 coral height
      else if (preset == 2) height = stage2Height; //L2 coral height
      else if (preset == 3) height = stage3Height; //L3 coral height
      else if (preset == 4) height = floorAlgaeHeight; //floor algae height
      else if (preset == 5) height = l1AlgaeHeight; //L1 algae height
      else if (preset == 6) height = l2AlgaeHeight; //L2 algae height
      else if (preset == 7) height = processorHeight; //processor height
      else if (preset == 8) height = humanPlayerHeight; //human player height
      else if (preset == 9) height = maxHeight; ///processor height
      
      //running the presets
       
      /*if (m_lEncoder.getPosition() < height) {
        if (m_lEncoder.getPosition() > height - 5 && m_lEncoder.getPosition() < height-1) {
          m_lMotor.set(MathUtil.applyDeadband((elevatorSpeed) * ((height - m_lEncoder.getPosition())/5), 0.05));
          m_rMotor.set(-MathUtil.applyDeadband((elevatorSpeed) * ((height - m_lEncoder.getPosition())/5), 0.05));
        }
        else {
          m_lMotor.set(elevatorSpeed);
          m_rMotor.set(-elevatorSpeed);
        }
      } 
      else if (m_lEncoder.getPosition() > height) {
        if (m_lEncoder.getPosition() < 2 && height == 0) {
          m_lMotor.set(-0.1);
          m_rMotor.set(0.1);
        }
        else if (m_lEncoder.getPosition() < height + 5) {
          m_lMotor.set(-MathUtil.applyDeadband((elevatorSpeed) * ((m_lEncoder.getPosition() - height)/5), 0.05));
          m_rMotor.set(MathUtil.applyDeadband((elevatorSpeed) * ((m_lEncoder.getPosition() - height)/5), 0.05));
        }
        else {
          m_lMotor.set(-elevatorSpeed);
          m_rMotor.set(elevatorSpeed);
        }
        if (m_lEncoder.getVelocity() >= -0.03 && m_lEncoder.getPosition() < 3 && height == 0) {
          resetEncoders();
        }
      }
        */
        
      
      m_lPID.setReference(m_heightProfile.getSetpoint(height, m_lEncoder.getPosition()), ControlType.kMAXMotionPositionControl);
      m_rPID.setReference(m_heightProfile.getSetpoint(-height, m_rEncoder.getPosition()), ControlType.kMAXMotionPositionControl);
    }
    else {
      m_lPID.setReference(m_lEncoder.getPosition(), ControlType.kPosition);
      m_rPID.setReference(m_rEncoder.getPosition(), ControlType.kPosition);
    }
    SmartDashboard.putNumber("PID", m_heightProfile.getSetpoint(height, m_lEncoder.getPosition()));
    SmartDashboard.putNumber("Left Integral Windup", m_lPID.getIAccum());
    SmartDashboard.putNumber("Right Integral Windup", m_rPID.getIAccum());
  }
}
