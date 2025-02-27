//Following REVLib's 2025 update, configurations to motors and encoders are now done through in a SparkBaseConfig object.
package frc.robot;

import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.SwerveModuleConstants;

public final class SwerveConfigs {
    /* 
     * Creates a SparkMaxConfig object for each motor in the swerve module.
     * 
     * These objects serve as "templates," to be later applied to each motor. 
     * Think of them as a list of settings that are automatically applied to each motor. 
     */
    public static final SparkMaxConfig m_driveConfig = new SparkMaxConfig();
    public static final SparkMaxConfig m_turnConfig = new SparkMaxConfig();

    //motion config object for drive motor
    public static final MAXMotionConfig m_motionConfig = new MAXMotionConfig();

    //executes once the class is loaded into memory
    static {
        //the lines below configure the SparkMaxConfig object for the drive motor
        m_driveConfig
            .idleMode(IdleMode.kBrake) //sets drive motor's idle mode into brake mode (locked in place)
            .smartCurrentLimit(SwerveModuleConstants.kDriveCurrentLimit); //limits the current that can be going through the drive motor
            
        m_driveConfig.encoder //configures the drive encoders
            .positionConversionFactor(SwerveModuleConstants.kDriveEncoderPositionFactor) //sets a conversion factor for position of the encoder
            .velocityConversionFactor(SwerveModuleConstants.kDriveEncoderVelocityFactor); //sets a conversion factor for velocity of the encoder

        m_driveConfig.closedLoop //configures the PID controller of the drive motor
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder) //sets primary sensor to be used for PID
            .pid(SwerveModuleConstants.kDriveP, SwerveModuleConstants.kDriveI, SwerveModuleConstants.kDriveD) //sets PID constants for the drive motor
            .velocityFF(SwerveModuleConstants.kDriveFF) //sets velocity feedforward gain
            .outputRange(SwerveModuleConstants.kDriveMinOutput, SwerveModuleConstants.kDriveMaxOutput); //sets range of drive motor output, currently set to -1 to 1

        m_driveConfig.closedLoop.maxMotion 
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal); //sets the motion profile of the motor to trapezoidal
        
            
        //the lines below configure the turn motor    
        m_turnConfig
            .idleMode(IdleMode.kCoast) //sets turn motor's idle mode into coast mode (free moving)
            .smartCurrentLimit(SwerveModuleConstants.kTurnCurrentLimit); //limits the current that can be going through the turn motor

        m_turnConfig.absoluteEncoder //configures the turn encoders
            .inverted(true) //inverts the turn motors
            .positionConversionFactor(SwerveModuleConstants.kTurnEncoderPositionFactor) //sets a conversion factor for the position of the encoder
            .velocityConversionFactor(SwerveModuleConstants.kTurnEncoderVelocityFactor); //sets a conversion factor for the velocity of the encoder
            
        m_turnConfig.closedLoop //configures the PID controller of the turn motor
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) //sets primary sensor to be used for PID
            .pid(SwerveModuleConstants.kTurnP, SwerveModuleConstants.kTurnI, SwerveModuleConstants.kTurnD) //sets PID constants for the turn motor
            .outputRange(SwerveModuleConstants.kTurnMinOutput, SwerveModuleConstants.kTurnMaxOutput) //sets range of turn motor output, current set to -1 to 1
            .positionWrappingEnabled(true) //allows the turn motor to "wrap," basically allowing it to go backwards if it is the shortest path to the desired state
            .positionWrappingInputRange(SwerveModuleConstants.kTurnEncoderPositionPIDMinInput, SwerveModuleConstants.kTurnEncoderPositionPIDMaxInput); //sett the input range for PID wrapping with position closed loop control, currently set from 0 to 2*pi
        
        //configures the motion profile for each motor
        m_motionConfig 
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal); //sets the motion profile to trapezoidal
    }
}
