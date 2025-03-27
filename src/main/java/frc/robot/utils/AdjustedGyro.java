// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 * A wrapper class designed to fix some of the issues with the current ADXRS450_Gyro.
 * 
 * This class stops stationary gyro drift while providing the necessary functionality of a gyro.
 */
public class AdjustedGyro extends SubsystemBase {
  //gyro that provides all the measurements necessary
  private ADXRS450_Gyro m_gyro;

  //angle at which the robot is first detected being stationary
  private double stoppedAngle;

  //amount of gyro drift that occurs while the robot is stationary
  private double accumulatedError;

  //boolean value for whether or not the robot is stationary
  private static boolean stopGyro;

  //prevents stoppedAngle from changing once it is set to true
  private boolean lock;

  /** Creates a new AdjustedGyro. 
   * 
   * @param gyro Gyro to be adjusted
   */
  public AdjustedGyro(ADXRS450_Gyro gyro) {
    //defaults to stationary robot
    stopGyro = true;

    //creates gyro
    this.m_gyro = gyro;
  }

  /**
   * Returns the adjusted gyro heading.
   * 
   * @return Gyro angle in degrees
   */
  public double getAngle() {
      return -(m_gyro.getAngle() - accumulatedError);
  }

  /**
   * Returns the adjusted gyro heading as a Rotation2d object.
   * 
   * @return Rotation2d for gyro angle
   */
  public Rotation2d getRotation2d() {
    return new Rotation2d(Units.degreesToRadians(getAngle()));
  } 

  /**
   * Resets the gyro.
   */
  public void reset() {
    m_gyro.reset();
    accumulatedError = 0;
    stoppedAngle = 0;
  }

  /**
   * Returns the rate of change (velocity) of the gyro.
   * 
   * @return Angular velocity in degrees per second
   */
  public double getRate() {
    return m_gyro.getRate();
  }

  /**
   * Sets whether or not the gyro should be "stopped" (when the robot is stationary).
   * 
   * @param stop Whether or not gyro should be stoppped
   */
  public static void setGyroStop(boolean stop) {
    stopGyro = stop;
  } 

  @Override
  public void periodic() {
    //when robot is stationary, lock the gyro measurement to one value
    if(stopGyro) 
    {
      //sets stoppedAngle
      if(!lock) {
        stoppedAngle = m_gyro.getAngle();
        lock = true; //prevents stoppedAngle from being set again
      }
      
      //stores gyro drift as a double value
      accumulatedError = m_gyro.getAngle() - stoppedAngle;
    }

    //when robot is not stationary, unlock stoppedAngle from being set and stop getting accumulated error
    else {
      lock = false;
    }
  }
}
