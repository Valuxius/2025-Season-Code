// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class AdjustedGyro extends SubsystemBase {
  private Joystick m_driverController = new Joystick(1);
  private ADXRS450_Gyro m_gyro;
  private double offsetPerFrame;
  private Timer timer = new Timer();
  private double accumulatedTime;
  private double accumulatedError;

  /** Creates a new AdjustedGyro. */
  public AdjustedGyro(ADXRS450_Gyro p_gyro) {
    m_gyro = p_gyro;
    timer.start();
  }

  @Override
  public void periodic() {
    while (Math.abs(m_driverController.getRawAxis(RobotConstants.kLeftXAxisPort)) <= 0.05 &&
           Math.abs(m_driverController.getRawAxis(RobotConstants.kLeftYAxisPort)) <= 0.05 &&
           Math.abs(m_driverController.getRawAxis(RobotConstants.kRightXAxisPort)) <= 0.05) {

    }
  }
}
