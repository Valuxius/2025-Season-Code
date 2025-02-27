//Following REVLib's 2025 update, configurations to motors and encoders are now done through in a SparkBaseConfig object.
package frc.robot;

import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.SwerveModuleConstants;

public final class SwerveConfigs {
    //creates a SparkMaxConfig object for each motor in the swerve module
    public static final SparkMaxConfig m_driveConfig = new SparkMaxConfig();
    public static final SparkMaxConfig m_turnConfig = new SparkMaxConfig();

    public static final MAXMotionConfig m_motionConfig = new MAXMotionConfig();

    //creates a MaxMotionConfig object for each motor in the swerve module

    //executes once the class is loaded into memory
    static {
        m_driveConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(SwerveModuleConstants.kDriveCurrentLimit);
            
        m_driveConfig.encoder
            .positionConversionFactor(SwerveModuleConstants.kDriveEncoderPositionFactor)
            .velocityConversionFactor(SwerveModuleConstants.kDriveEncoderVelocityFactor);

        m_driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(SwerveModuleConstants.kDriveP, SwerveModuleConstants.kDriveI, SwerveModuleConstants.kDriveD)
            .velocityFF(SwerveModuleConstants.kDriveFF)
            .outputRange(SwerveModuleConstants.kDriveMinOutput, SwerveModuleConstants.kDriveMaxOutput);

        m_driveConfig.closedLoop.maxMotion
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
            
        m_turnConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(SwerveModuleConstants.kTurnCurrentLimit);

        m_turnConfig.absoluteEncoder
            .inverted(true)
            .positionConversionFactor(SwerveModuleConstants.kTurnEncoderPositionFactor)
            .velocityConversionFactor(SwerveModuleConstants.kTurnEncoderVelocityFactor);
            
        m_turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(SwerveModuleConstants.kTurnP, SwerveModuleConstants.kTurnI, SwerveModuleConstants.kTurnD)
            .outputRange(SwerveModuleConstants.kTurnMinOutput, SwerveModuleConstants.kTurnMaxOutput)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(SwerveModuleConstants.kTurnEncoderPositionPIDMinInput, SwerveModuleConstants.kTurnEncoderPositionPIDMaxInput);
        
        m_motionConfig
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    }
}
