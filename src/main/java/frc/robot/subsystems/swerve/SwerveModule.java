package frc.robot.subsystems.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.MotorConfigs;
import frc.robot.utils.TrapezoidalVelocityControl;

public class SwerveModule {
    private final TrapezoidalVelocityControl m_driveProfile;
    
    //initializing the motors
    private final SparkMax m_driveMotor;
    private final SparkMax m_turnMotor;

    //initializing the encoders
    private final RelativeEncoder m_driveEncoder;
    private final AbsoluteEncoder m_turnEncoder;

    //initializing the closed loop controllers (also known as PID controllers)
    private final SparkClosedLoopController m_drivePID;
    private final SparkClosedLoopController m_turnPID;

    private double m_angleOffset; //default chassis offset aka the angle of the module from its calibrated position to being stright
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d()); //sets the swerve module's velocity to 0 and the angle to 0 when the code is initialized

    private SlewRateLimiter filter = new SlewRateLimiter(15);

    public SwerveModule(int p_driveID, int p_turnID, double p_angleOffset) {
        //creates a new trapezoid profile (custom) to control drive motor velocity
        m_driveProfile = new TrapezoidalVelocityControl(54, 54);

        //setting drive and turn motors for each swerve module
        m_driveMotor = new SparkMax(p_driveID, MotorType.kBrushless);
        m_turnMotor = new SparkMax(p_turnID, MotorType.kBrushless);

        //sets the encoders for the drive and turn motors
        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnEncoder = m_turnMotor.getAbsoluteEncoder(); //turn motor uses absolute encoder so we can always measure the rotation of the wheel

        //setting the PID controllers for the motors
        m_drivePID = m_driveMotor.getClosedLoopController();
        m_turnPID = m_turnMotor.getClosedLoopController();

        //resets everything
        m_angleOffset = p_angleOffset;
        m_driveEncoder.setPosition(0);
        m_desiredState.angle = new Rotation2d(m_turnEncoder.getPosition());

        //configure the swerve module with the settings defined in MotorConfigs.java
        m_driveMotor.configure(MotorConfigs.m_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_turnMotor.configure(MotorConfigs.m_turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Resets the drive encoder.
     */
    public void resetEncoders() {
        m_driveEncoder.setPosition(0); //sets the drive encoder position to zero
    }

    /**
     * Returns the current position and angle of the module.
     * 
     * @return Position and angle of the motor
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(), //position of the drive motor
            new Rotation2d(m_turnEncoder.getPosition() - m_angleOffset)); //position of the turn motor, offset by the chassis angular offset
    }

    /**
     * Returns the current speed and angle of the module.
     * 
     * @return Speed and angle of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_driveEncoder.getVelocity(), //velocity of the drive motor
            new Rotation2d(m_turnEncoder.getPosition() - m_angleOffset)); //position of the turn motor, offset by the chassis angular offset
    }


    /**
     * Sets the speed and angle of the module.
     * 
     * @param desiredState Desired speed and angle
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        //creates a state for what we want
        SwerveModuleState correctedDesiredState = new SwerveModuleState();

        //stores our desired speed
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond; 

        //adds the offset to the desired angle and stores the angle of the module
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_angleOffset)); 

        //minimizes the angle change needed to set the correct angle
        correctedDesiredState.optimize(new Rotation2d(m_turnEncoder.getPosition()));

        //sets the state of the module to the state we created
        m_drivePID.setReference(m_driveProfile.getSetpoint(correctedDesiredState.speedMetersPerSecond, m_driveEncoder.getVelocity()), ControlType.kVelocity);
        m_turnPID.setReference(filter.calculate(correctedDesiredState.angle.getRadians()), ControlType.kPosition);

        m_desiredState = desiredState;
    }
}
