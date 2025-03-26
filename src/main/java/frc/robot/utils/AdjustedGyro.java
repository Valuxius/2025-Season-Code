// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AdjustedGyro extends SubsystemBase {
  //private Joystick m_driverController = new Joystick(1);
  public ADXRS450_Gyro m_gyro;
  private Timer timer = new Timer();
  private double stoppedAngle;
  private double accumulatedError;
  private static boolean stopGyro;
  private boolean lock;

  /** Creates a new AdjustedGyro. */
  public AdjustedGyro(ADXRS450_Gyro p_gyro) {
    stopGyro = true;
    this.m_gyro = p_gyro;
    timer.start();
  }

  public double getAngle() {
      return -(m_gyro.getAngle() - accumulatedError);
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(Units.degreesToRadians(getAngle()));
  } 

  public void reset() {
    m_gyro.reset();
    accumulatedError = 0;
    stoppedAngle = 0;
  }

  public double getRate() {
    return m_gyro.getRate();
  }

  public static void setGyroStop(boolean stop) {
    stopGyro = stop;
  } 

  @Override
  public void periodic() {
    if(stopGyro) 
    {
      if(!lock) {
        stoppedAngle = m_gyro.getAngle();
        lock = true;
      }
      accumulatedError = m_gyro.getAngle() - stoppedAngle;
    }
    else {
      lock = false;
    }
  }
}
