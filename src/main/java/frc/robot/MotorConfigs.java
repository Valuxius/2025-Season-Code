//Following REVLib's 2025 update, configurations to motors and encoders are now done through in a SparkBaseConfig object.
package frc.robot;

import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.RobotConstants;

public final class MotorConfigs {
    /* 
     * Creates a SparkMaxConfig object for each motor in the swerve module.
     * 
     * These objects serve as "templates," to be later applied to each motor. 
     * Think of them as a list of settings that are automatically applied to each motor. 
     */
    public static final SparkMaxConfig m_driveConfig = new SparkMaxConfig();
    public static final SparkMaxConfig m_turnConfig = new SparkMaxConfig();

    public static final SparkMaxConfig m_climbConfig = new SparkMaxConfig();
    public static final SparkMaxConfig m_elevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig m_shooterConfig = new SparkMaxConfig();

    //motion config object for drive motor
    public static final MAXMotionConfig m_motionConfig = new MAXMotionConfig();

    //executes once the class is loaded into memory
    static {
        //configures the motion profile for each motor
        m_motionConfig 
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal); //sets the motion profile to trapezoidal

        //the lines below configure the SparkMaxConfig object for the drive motor
        m_driveConfig
            .idleMode(IdleMode.kCoast) //sets drive motor's idle mode into brake mode (locked in place)
            .smartCurrentLimit(RobotConstants.kDriveCurrentLimit); //limits the current that can be going through the drive motor

        m_driveConfig.encoder //configures the drive encoders
            .positionConversionFactor(RobotConstants.kDriveEncoderPositionFactor) //sets a conversion factor for position of the encoder
            .velocityConversionFactor(RobotConstants.kDriveEncoderVelocityFactor); //sets a conversion factor for velocity of the encoder

        m_driveConfig.closedLoop //configures the PID controller of the drive motor
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder) //sets primary sensor to be used for PID
            .pid(RobotConstants.kDriveP, RobotConstants.kDriveI, RobotConstants.kDriveD) //sets PID constants for the drive motor
            .velocityFF(RobotConstants.kDriveFF) //sets velocity feedforward gain
            .outputRange(RobotConstants.kDriveMinOutput, RobotConstants.kDriveMaxOutput); //sets range of drive motor output, currently set to -1 to 1

        m_driveConfig.closedLoop.maxMotion 
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal); //sets the motion profile of the motor to trapezoidal
        
            
        //the lines below configure the turn motor    
        m_turnConfig
            .idleMode(IdleMode.kCoast) //sets turn motor's idle mode into  mode (free moving)
            .smartCurrentLimit(RobotConstants.kTurnCurrentLimit); //limits the current that can be going through the turn motor

        m_turnConfig.absoluteEncoder //configures the turn encoders
            .inverted(true) //inverts the turn motors
            .positionConversionFactor(RobotConstants.kTurnEncoderPositionFactor) //sets a conversion factor for the position of the encoder
            .velocityConversionFactor(RobotConstants.kTurnEncoderVelocityFactor); //sets a conversion factor for the velocity of the encoder
            
        m_turnConfig.closedLoop //configures the PID controller of the turn motor
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) //sets primary sensor to be used for PID
            .pid(RobotConstants.kTurnP, RobotConstants.kTurnI, RobotConstants.kTurnD) //sets PID constants for the turn motor
            .outputRange(RobotConstants.kTurnMinOutput, RobotConstants.kTurnMaxOutput) //sets range of turn motor output, current set to -1 to 1
            .positionWrappingEnabled(true) //allows the turn motor to "wrap," basically allowing it to go backwards if it is the shortest path to the desired state
            .positionWrappingInputRange(RobotConstants.kTurnEncoderPositionPIDMinInput, RobotConstants.kTurnEncoderPositionPIDMaxInput); //set the input range for PID wrapping with position closed loop control, currently set from 0 to 2*pi

        m_turnConfig.closedLoop.maxMotion
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal); 

        //the lines below configure the SparkMaxConfig object for the climb motor
        m_climbConfig
            .idleMode(IdleMode.kBrake);

        m_climbConfig.closedLoop
            .pid(RobotConstants.kClimbP, RobotConstants.kClimbI, RobotConstants.kClimbD);
        
        //the lines below configure the SparkMaxConfig object for the elevator motor
        m_elevatorConfig
            .idleMode(IdleMode.kBrake);
        
        m_elevatorConfig.closedLoop
            .pid(RobotConstants.kElevatorP, RobotConstants.kElevatorI, RobotConstants.kElevatorD);

        //the lines below configure the SparkMaxConfig object for the shooter motor
        m_shooterConfig
            .idleMode(IdleMode.kBrake);
        
        m_shooterConfig.closedLoop
            .pid(RobotConstants.kShooterP, RobotConstants.kShooterI, RobotConstants.kShooterD);
    }
}
