// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class TrapezoidalPositionControl {
    private final double maxVelocity;
    private final double deadband;

    public TrapezoidalPositionControl(double p_maxVelocity, double p_deadband) {
        maxVelocity = p_maxVelocity;
        deadband = p_deadband;
    }

    public double getSetpoint(double desiredPosition, double previousPosition) {
        if (maxVelocity / 50 < Math.abs(desiredPosition - previousPosition)) {
            if (Math.abs(desiredPosition - previousPosition) < deadband) return desiredPosition;
            else return (desiredPosition - previousPosition > 0) ? 
                previousPosition + (maxVelocity/50) : 
                previousPosition - (maxVelocity/50);
        }
        else return desiredPosition;
    }
}
