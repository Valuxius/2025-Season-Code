// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand; //instance field for autonomous command

  private final RobotContainer m_robotContainer; //initializes the robot container

  public Robot() {
    m_robotContainer = new RobotContainer(); //creates the robot container
    /*for (int port = 5800; port <= 5809; port++) { //port forwards limelight ports to be able to be accessed through usb
      PortForwarder.add(port, "limelight-front.local", port);
    }*/
  }

  //runs periodically while the robot is active
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  //runs when the robot is disabled
  @Override
  public void disabledInit() {}

  //runs periodically while the robot is disabled
  @Override
  public void disabledPeriodic() {}

  //runs after robot leaves disabled state
  @Override
  public void disabledExit() {}

  //runs on start of autonomous
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(); //fetches autonomous command to be run

    //if autonomous command exists, schedule it. otherwise, do nothing
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  //runs periodically during autonomous
  @Override
  public void autonomousPeriodic() {}

  //runs after robot leaves autonomous state
  @Override
  public void autonomousExit() {}

  //runs on start of teleop
  @Override
  public void teleopInit() {
    //if autonomous command exists, cancel it
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  //runs periodically during teleop
  @Override
  public void teleopPeriodic() {}

  //runs after robot leaves teleop state
  @Override
  public void teleopExit() {}

  //runs on start of test 
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  //runs periodically during test
  @Override
  public void testPeriodic() {}

  //runs after robot leaves test state
  @Override
  public void testExit() {}
}
