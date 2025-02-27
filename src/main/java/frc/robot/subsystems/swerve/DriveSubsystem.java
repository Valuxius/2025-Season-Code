// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  //configuring the swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveMotorPort, 
    DriveConstants.kFrontLeftTurnMotorPort, 
    DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveMotorPort, 
    DriveConstants.kFrontRightTurnMotorPort, 
    DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveMotorPort, 
    DriveConstants.kBackLeftTurnMotorPort, 
    DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_backRight = new SwerveModule(
    DriveConstants.kBackRightDriveMotorPort, 
    DriveConstants.kBackRightTurnMotorPort, 
    DriveConstants.kBackRightChassisAngularOffset);

  //initialize the gyro
  public final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  //storing the positions of the swerve modules
  private final SwerveModulePosition[] m_swervePositions = getPositions();

  //odometry object to track the robots pose on the field
  private final SwerveDriveOdometry m_driveOdometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(), m_swervePositions);

  //poseEstimator object which tracks the robots pose on the field (similar to odometry object but can use vision data for more accurate tracking)
  private final SwerveDrivePoseEstimator m_poseEstimator =
    new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        m_gyro.getRotation2d(),
        getPositions(),
        new Pose2d(), //below - 1x3 matrix of the form (x,y,theta) in meters and radians
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), //stddev of state-space control pose estimate (wpilib) //increase values if you want to trust this estimate less
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); //stddev of vision pose estimate (limelight) //increase values if you want to trust this estimate less

  //Field widget for SmartDashboard
  public final Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    RobotConfig robotConfig = null;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    //supplies methods for pathplanner to use for position estimation and auto
    AutoBuilder.configure( 
      this::getPose, //get position method
      (pose) -> {
        resetPose(pose);
        m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getPositions(), pose);
      },
      this::getCurrentSpeeds,
      (speeds, feedforwards) -> autoDrive(speeds),
      new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0)
      ),
      robotConfig,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );
  }

  /**
   * Drives the robot.
   * 
   * @param xSpeed Speed in the x direction
   * @param ySpeed Speed in the y direction
   * @param rot Angular speed
   * @param fieldRelative Whether the provided x and y speeds are relative to the field
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the controller input into units for the drive train
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      ChassisSpeeds.discretize(fieldRelative ?
        ChassisSpeeds.fromFieldRelativeSpeeds( //converts field relative input into chassis speeds
          xSpeedDelivered, 
          ySpeedDelivered, 
          rotDelivered, 
          m_gyro.getRotation2d())
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered), //uses raw controller input as chassis speeds
      DriveConstants.kDriverPeriod)); 

    // caps the wheel speeds so they don't go faster then they should
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    
    //sets the desired states of the swerve modules
    
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void autoDrive(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      ChassisSpeeds.discretize(
        speeds, DriveConstants.kDriverPeriod));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Locks the robot in place by setting the wheels in an X formation.
   */
  public void setX() {
    m_frontLeft.setDesiredState(
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backLeft.setDesiredState(
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backRight.setDesiredState(
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

   /** Resets the drive encoders to currently read a position of 0. */
   public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  /**
   * Gets the estimated position of the robot on the field.
   * 
   * @return Position of the robot on the field
   */
  public Pose2d getPose() {
    return m_driveOdometry.getPoseMeters();
  }

  /**
   * Resets the position of the odometry to the Pose2d supplied.
   * 
   * @param pose Position to reset to
   */
  public void resetPose(Pose2d pose) {
    m_driveOdometry.resetPosition(m_gyro.getRotation2d(), getPositions(), pose);
  }

  /**
   * Gets the current position of all four swerve modules
   * 
   * @return Positions of all swerve modules
   */
  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()};
  }

  /**
   * Gets the current state of all four swerve modules
   * 
   * @return States of all swerve modules
   */
  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_backLeft.getState(),
      m_backRight.getState()};
  }

  /**
   * Gets the chassis speed representing the speed the wheels are moving at
   * 
   * @return Speed of the robot
   */
  public ChassisSpeeds getCurrentSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getStates());
  }

  public Command resetGyro() {
    return new InstantCommand(() -> {m_gyro.reset();}, this);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() { 
    //updates the odometry every scheduled frame
    m_driveOdometry.update( 
      m_gyro.getRotation2d(),
      getPositions()
    );

    //updates the pose estimater every scheduled frame
    m_poseEstimator.update(
      m_gyro.getRotation2d(), 
      getPositions()
    );
  }

}
