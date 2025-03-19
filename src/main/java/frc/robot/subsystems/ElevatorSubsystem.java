// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorConfigs;
import frc.robot.Constants.RobotConstants;

public class ElevatorSubsystem extends SubsystemBase {
  //initializing the motors
  private final SparkMax m_lMotor;
  private final SparkMax m_rMotor;

  //initializing the encoders
  private final RelativeEncoder m_lEncoder;

  private final TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goalState;
  private TrapezoidProfile.State currentState;
  private final TrapezoidProfile profile;

  private double currentPos;

  //variables for the presets
  private double preset = 0;
  private boolean lock = true; //will toggle to false when needed, locks the elevator at preset if set to true
  
  //height instance variable
  private double height = 0;
  
  //heights for presets
  //IMPORTANT FOR LATER - OFFSET +0.5 FOR DEADBAND
  private double stage1Height = 4; //L1 Coral Height
  private double stage2Height = 17; //L2 Coral Height
  private double stage3Height = 31; //L3 Coral Height
  private double maxHeight = 40.5; //Max Height

  private double floorAlgaeHeight = 10; //Floor Algae Height
  private double l1AlgaeHeight = 0; //L1 Algae Height
  private double l2AlgaeHeight = 26.5; //L2 Algae Height

  private double processorHeight = 2; //Processor Height

  private double humanPlayerHeight = 14; //Human Player Height

  //speed for presets (0 - 1)
  private double elevatorSpeed = 0.6;
  
  private final ProfiledPIDController m_PID;

  /*public enum ElevatorPosition {
    DOWN(0),
    STAGE_1(ElevatorConstants.L1),
    STAGE_2(ElevatorConstants.L2),
    STAGE_3(ElevatorConstants.L3),
    POSITION_4(ElevatorConstants.L4);

    public final double positionInches;
    
    ElevatorPosition(double positionInches) {
        this.positionInches = positionInches;
    }
}*/

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(int p_leftID, int p_rightID) {
    //creating the motors for the subsystem
    m_lMotor = new SparkMax(p_leftID, MotorType.kBrushless);
    m_rMotor = new SparkMax(p_rightID, MotorType.kBrushless);

    //creating the encoders for the subsystem
    m_lEncoder = m_lMotor.getEncoder();

    //resets the encoders
    m_lEncoder.setPosition(0);

    //configures the motors with PID controls
    m_lMotor.configure(MotorConfigs.m_elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig m_rConfig = MotorConfigs.m_elevatorConfig;
    m_rConfig.follow(RobotConstants.kLeftElevatorMotorPort, true);

    m_rMotor.configure(m_rConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    constraints = new TrapezoidProfile.Constraints(15, 30);
    m_PID = new ProfiledPIDController(
      RobotConstants.kElevatorP, RobotConstants.kElevatorI, RobotConstants.kElevatorD, constraints, 0.02);
    currentState = new TrapezoidProfile.State(0, 0);
    goalState = new TrapezoidProfile.State(0, 0);
    profile = new TrapezoidProfile(constraints);
  }

  /**
   * Moves the elevator up.
   * 
   * @param speed Speed of ascent, 0 - 1
   */
  public void ascend(double speed) {
    preset = 10;
    m_PID.reset(m_lEncoder.getPosition());
    currentState = new TrapezoidProfile.State(m_lEncoder.getPosition(), 0);
    goalState = new TrapezoidProfile.State(m_lEncoder.getPosition(), 0);
    height = m_lEncoder.getPosition();
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

    //posts the data to SmartDashboard (for troubleshooting)
    SmartDashboard.putNumber("Speed", newSpeed);
  }

  /**
   * Moves the elevator down.
   * 
   * @param speed Speed of descent, 0 - 1
   */
  public void descend(double speed) {
    preset = 10;
    m_PID.reset(m_lEncoder.getPosition());
    currentState = new TrapezoidProfile.State(m_lEncoder.getPosition(), 0);
    goalState = new TrapezoidProfile.State(m_lEncoder.getPosition(), 0);
    height = m_lEncoder.getPosition();
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
  }

  /**
   * Resets the elevator encoders.
   */
  public void resetEncoder() {
    m_lEncoder.setPosition(0);
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

  public double calculateFF(TrapezoidProfile.State state) {
    return RobotConstants.kElevatorS * Math.signum(state.velocity) +
           RobotConstants.kElevatorG +
           RobotConstants.kElevatorV * state.velocity;
  }
  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    currentPos = m_lEncoder.getPosition();
    
    currentState = profile.calculate(0.02, currentState, goalState);
    //puts encoder position on SmartDashboard for troubleshooting
    SmartDashboard.putNumber("Elevator Encoder", m_lEncoder.getPosition());
    
    goalState = new TrapezoidProfile.State(height, 0);
    m_PID.setTolerance(0.25);
    double output = m_PID.calculate(m_lEncoder.getPosition(), currentState.position);
    double ff = calculateFF(currentState);
    if (m_lEncoder.getPosition() < 9 && currentState.velocity < 0 && height == 0) {
      m_lMotor.set(-0.75 * m_lEncoder.getPosition()/8);
    } 
    else if (Math.abs(m_lEncoder.getPosition() - height) < 3) {
      m_lMotor.set(MathUtil.applyDeadband(
        (height > m_lEncoder.getPosition()) ?
        0.75 * (height - m_lEncoder.getPosition())/3 :
        -0.75 * (m_lEncoder.getPosition() - height)/3,
        0.05
        )
      );
    }
    else m_lMotor.set(MathUtil.clamp(output+ff, -0.75, 0.75));
    SmartDashboard.putNumber("ff", ff);
    SmartDashboard.putNumber("output", output);
  }
}
