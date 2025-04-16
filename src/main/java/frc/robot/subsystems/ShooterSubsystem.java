// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.MotorConfigs;

public class ShooterSubsystem extends SubsystemBase {
  //initializing the motors
  private final SparkMax m_shooterMotor;
  private final SparkMax m_rotationMotor;

  //initializing the encoder
  private final RelativeEncoder m_rotationEncoder;

  //initalizing the PID controllers 
  private final ProfiledPIDController m_rotationPID;

  private final TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goalState;
  private TrapezoidProfile.State currentState;
  private final TrapezoidProfile profile;

  //variable for preset
  private double preset = 0;

  //angle variables (0 is bottom)
  private double netPreset = 9.5 * 1.8;
  private double algaePreset = 10;
  private double floorPreset = 8 * 1.8;
  private double l1coralPreset = 4 * 1.8;
  private double l2coralPreset = 2 * 1.8;
  private double l3coralPreset = 1 * 1.8;
  private double humanPlayerPreset = 2 * 1.8;
  private double processorPreset = 5;
  private double startPreset = -18;

  //rotation variable
  private double rotation = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(int p_shooterID, int p_rotationID) {
    //creating the motors
    m_shooterMotor = new SparkMax(p_shooterID, MotorType.kBrushless);
    m_rotationMotor = new SparkMax(p_rotationID, MotorType.kBrushless);

    //creating the encoders
    m_rotationEncoder = m_rotationMotor.getEncoder();

    //resets the encoders
    m_rotationEncoder.setPosition(0);

    //adds a max velocity and max acceleration (in encoder rotations)
    constraints = new TrapezoidProfile.Constraints(10, 10);

    //makes a new PID with the trapezoidal constraints
    m_rotationPID = new ProfiledPIDController(
      RobotConstants.kShooterP, RobotConstants.kShooterI, RobotConstants.kShooterD, constraints, 0.02);

    //disables I accumulation when it hits 0.25
    m_rotationPID.setIZone(0.25);
    
    //default states
    currentState = new TrapezoidProfile.State(0, 0);
    goalState = new TrapezoidProfile.State(0, 0);
    
    //creates trapezoidal profile to calculate the next step for PID
    profile = new TrapezoidProfile(constraints);

    //configures the PIDs for the motors
    m_shooterMotor.configure(MotorConfigs.m_shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rotationMotor.configure(MotorConfigs.m_shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //sets tolerance for feedback control
    m_rotationPID.setTolerance(0.1);
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
   * Rotates the shooter.
   * 
   * @param speed Speed to rotate at, 0 - 1.
   */
  public void rotate(double speed) {
    //disables PID control
    m_rotationPID.reset(m_rotationEncoder.getPosition());
    
    //disables trapezoidal calculations
    currentState = new TrapezoidProfile.State(m_rotationEncoder.getPosition(), 0);
    goalState = new TrapezoidProfile.State(m_rotationEncoder.getPosition(), 0);

    //locks rotation to current setpoint
    rotation = m_rotationEncoder.getPosition();

    double newSpeed = speed;

    //slows down rotation when moving down to the bottom
    // if (m_rotationEncoder.getPosition() < 7.2 && speed < 0) {
    //   newSpeed = speed * (m_rotationEncoder.getPosition()/7.2);
    // } 
    
    //slows down rotation when moving up
    if (m_rotationEncoder.getPosition() > 15 && speed > 0) {
      newSpeed = speed * ((18 - m_rotationEncoder.getPosition())/3);
    }
    
    m_rotationMotor.set(newSpeed);
  }

  /**
   * Sets the preset of the shooter angle.
   * 
   * @param preset Preset number
   */
  public void setPreset(int preset) {
    this.preset = preset;

    if (preset == 0) rotation = 0;
    else if (preset == 1) rotation = netPreset;
    else if (preset == 2) rotation = algaePreset;
    else if (preset == 3) rotation = floorPreset;
    else if (preset == 4) rotation = l1coralPreset;
    else if (preset == 5) rotation = l2coralPreset;
    else if (preset == 6) rotation = l3coralPreset;
    else if (preset == 7) rotation = humanPlayerPreset;
    else if (preset == 8) rotation = processorPreset;
    else if (preset == 9) rotation = startPreset;
  }

  /**
   * Gets the shooter's rotation encoder.
   * 
   * @return Shooter rotation encoder
   */
  public RelativeEncoder getRotationEncoder() {
    return m_rotationEncoder;
  }

  /**
   * Zeroes the shooter rotation encoder.
   */
  public void resetEncoder() {
    m_rotationEncoder.setPosition(0);
  }

  /**
   * Calculates the necessary feedforward to control the shooter.
   * 
   * @param state Current state of the shooter (position, velocity)
   * @return Feedforward value to set to motors
   */
  public double calculateFF(TrapezoidProfile.State state) {
    return RobotConstants.kShooterS * Math.signum(state.velocity) +
           RobotConstants.kShooterG +
           RobotConstants.kShooterV * state.velocity;
  }

  public boolean isMoving() {
    return Math.abs(m_rotationEncoder.getVelocity()) > 0.15 && Math.abs(m_rotationEncoder.getPosition() - rotation) > 0.5;
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    //sets goal setpoint for trapezoidal calculation
    goalState = new TrapezoidProfile.State(rotation, 0);
    
    //calculates the next "step" for trapezoidal motion
    currentState = profile.calculate(0.02, currentState, goalState);

    //calculates feedforward output
    double ff = MathUtil.clamp(calculateFF(currentState), -0.2, 0.2);

    //adjusts for feedforward error using a feedback controller
    double output = m_rotationPID.calculate(m_rotationEncoder.getPosition(), currentState.position);

    //moves the shooter to max height when it hits within 0.5 of the max rotation
    if (m_rotationEncoder.getPosition() > 16.5) {
      m_rotationMotor.set(ff * ((17 - m_rotationEncoder.getPosition())/0.5));
    }

    //resets our encoders from starting config
    else if (m_rotationEncoder.getPosition() > -18 && preset == 9) {
      if (rotation == -18 && m_rotationEncoder.getPosition() < -15 && Math.abs(m_rotationEncoder.getVelocity()) < 0.005) {
        m_rotationMotor.set(0);
        resetEncoder();
        preset = 0;
      }
      else m_rotationMotor.set(-0.05);//m_rotationMotor.set(-0.3 * (m_rotationEncoder.getPosition() + 12)/12);
    }

    //slows down output near the bottom of the shooter 
    else if (Math.abs(m_rotationEncoder.getPosition() - rotation) < 2.7 && currentState.velocity != 0) {
      m_rotationMotor.set(
        (rotation > m_rotationEncoder.getPosition()) ?
        0.36 * ((rotation - m_rotationEncoder.getPosition())/2.7) + RobotConstants.kShooterG :
        -0.36 * ((m_rotationEncoder.getPosition() - rotation)/2.7) + RobotConstants.kShooterG
      );
    }

    //uses feedforward and PID control to set a setpoint
    else m_rotationMotor.set(ff+output);

    //detects when the shooter is at the bottom and zeroes the encoders
    if (rotation == 0 && m_rotationEncoder.getPosition() < 0.9 && Math.abs(m_rotationEncoder.getVelocity()) < 0.005) {
      resetEncoder();
    }

    //posts the shooter encoder value to Shuffleboard
    SmartDashboard.putNumber("ShooterEncoder", m_rotationEncoder.getPosition());
  }
}
