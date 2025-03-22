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
  private final RelativeEncoder m_Encoder;

  private final TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goalState;
  private TrapezoidProfile.State currentState;
  private final TrapezoidProfile profile;

  //variables for the presets
  private double preset = 0;
  
  //height instance variable
  private double height = 0;
  
  //heights for presets
  //IMPORTANT FOR LATER - OFFSET +0.5 FOR DEADBAND
  private double stage1Height = 4; //L1 Coral Height
  private double stage2Height = 17; //L2 Coral Height
  private double stage3Height = 31; //L3 Coral Height
  private double maxHeight = 46; //Max Height

  private double floorAlgaeHeight = 10; //Floor Algae Height
  private double l1AlgaeHeight = 7; //L1 Algae Height
  private double l2AlgaeHeight = 20; //L2 Algae Height

  private double processorHeight = 0; //Processor Height

  private double humanPlayerHeight = 7; //Human Player Height

  private final ProfiledPIDController m_PID;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(int p_leftID, int p_rightID) {
    //creating the motors for the subsystem
    m_lMotor = new SparkMax(p_leftID, MotorType.kBrushless);
    m_rMotor = new SparkMax(p_rightID, MotorType.kBrushless);

    //creating the encoder for the subsystem
    m_Encoder = m_lMotor.getEncoder();

    //resets the encoder
    m_Encoder.setPosition(0);

    //configures the motors with PID controls
    m_lMotor.configure(MotorConfigs.m_elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //creates new config telling the right motor to follow the left motor (inverted)
    SparkMaxConfig m_rConfig = MotorConfigs.m_elevatorConfig;
    m_rConfig.follow(RobotConstants.kLeftElevatorMotorPort, true);

    //applies previous configuration
    m_rMotor.configure(m_rConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //adds a max velocity and max acceleration (in encoder rotations)
    constraints = new TrapezoidProfile.Constraints(15, 30);

    //makes a new PID with the trapezoidal constraints
    m_PID = new ProfiledPIDController(
      RobotConstants.kElevatorP, RobotConstants.kElevatorI, RobotConstants.kElevatorD, constraints, 0.02);
    
    //default states
    currentState = new TrapezoidProfile.State(0, 0);
    goalState = new TrapezoidProfile.State(0, 0);
    
    //creates trapezoidal profile to calculate the next step for PID
    profile = new TrapezoidProfile(constraints);
  }

  /**
   * Moves the elevator up.
   * 
   * @param speed Speed of ascent, 0 - 1
   */
  public void ascend(double speed) {
    //disables presets
    preset = 10; 
    
    //disables PID control while moving
    m_PID.reset(m_Encoder.getPosition()); 

    //disables trapezoidal calculations
    currentState = new TrapezoidProfile.State(m_Encoder.getPosition(), 0);
    goalState = new TrapezoidProfile.State(m_Encoder.getPosition(), 0);
    
    //locks target height to current height
    height = m_Encoder.getPosition();

    //creates new variable for adjusted speed
    double newSpeed = speed; 

    //reduces speed near the top as a "stop"
    if (m_Encoder.getPosition() > maxHeight - 3) { //starts reducing speed after encoder position reaches 3 below maxHeight
      newSpeed = Math.abs( //absolute value
        (((speed)/3)*(m_Encoder.getPosition() - maxHeight)) //linearly decreases speed to 0 at maxHeight
      ); 
    } 

    //sets the motor speeds
    m_lMotor.set(MathUtil.applyDeadband(newSpeed, 0.1));
  }

  /**
   * Moves the elevator down.
   * 
   * @param speed Speed of descent, 0 - 1
   */
  public void descend(double speed) {
    //disables presets
    preset = 10;

    //disables PID while ascending
    m_PID.reset(m_Encoder.getPosition());

    //disables trapezoidal calculations
    currentState = new TrapezoidProfile.State(m_Encoder.getPosition(), 0);
    goalState = new TrapezoidProfile.State(m_Encoder.getPosition(), 0);

    //locks target height
    height = m_Encoder.getPosition();

    //creates new variable for adjusted speed
    double newSpeed = speed;

    //reduces speed near the bottom as a "stop"
    if (m_Encoder.getPosition() < 1) { //minimum speed at position 1
      newSpeed = 0.05;
    } else if (m_Encoder.getPosition() < 5) { //starts reducing speed at position 5
      newSpeed = 0.05 + //minimum speed
      (((speed-0.05)/5)*(m_Encoder.getPosition() - 1)); //linearly decreases speed to 0.1 at position 1 (5 - 4)
    }

    //sets motor speeds
    m_lMotor.set(-newSpeed);
  }

  /**
   * Sets the elevator stage.
   * 
   * @param stage Stage to be set to.
   */
  public void setPreset(double preset) {
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
    m_Encoder.setPosition(0);
  }

  /**
   * Gets the left elevator encoder. 
   * 
   * @return Elevator left encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return m_Encoder;
  }

  /**
   * Gets the right elevator encoder.
   * 
   * @return Elevator right encoder
   */
  public RelativeEncoder getRightEncoder() {
    return m_Encoder;
  }

  /**
   * Calculates the necessary feedforward to control the elevator.
   * 
   * @param state Current state of the elevator (position, velocity)
   * @return Feedforward value to set to motors
   */
  public double calculateFF(TrapezoidProfile.State state) {
    return RobotConstants.kElevatorS * Math.signum(state.velocity) +
           RobotConstants.kElevatorG +
           RobotConstants.kElevatorV * state.velocity;
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() { 
    //calculates the next "step" in the trapezoidal motion
    currentState = profile.calculate(0.02, currentState, goalState);

    //puts encoder position on SmartDashboard for troubleshooting
    SmartDashboard.putNumber("Elevator Encoder", m_Encoder.getPosition());
    
    goalState = new TrapezoidProfile.State(height, 0);
    m_PID.setTolerance(0.25);
    double output = m_PID.calculate(m_Encoder.getPosition(), currentState.position);
    double ff = calculateFF(currentState);
    if (m_Encoder.getPosition() < 9 && currentState.velocity < 0 && height == 0) {
      m_lMotor.set(-0.75 * m_Encoder.getPosition()/9);
    } 
    else if (Math.abs(m_Encoder.getPosition() - height) < 3) {
      m_lMotor.set(MathUtil.applyDeadband(
        (height > m_Encoder.getPosition()) ?
        0.75 * (height - m_Encoder.getPosition())/3 :
        -0.75 * (m_Encoder.getPosition() - height)/3,
        0.05
        )
      );
    }
    else m_lMotor.set(MathUtil.clamp(output+ff, -0.75, 0.75));
  }
}
