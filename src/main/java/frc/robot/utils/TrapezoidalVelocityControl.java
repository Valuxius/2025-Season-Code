// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class TrapezoidalVelocityControl {
    private final double maxDeceleration;
    private final double maxAcceleration;

    public TrapezoidalVelocityControl(double p_maxDeceleration, double p_maxAcceleration) {
        maxAcceleration = p_maxAcceleration;
        maxDeceleration = p_maxDeceleration;
    }

    public double getSetpoint(double desiredVelocity, double previousVelocity) {
        if (Math.abs(desiredVelocity) < Math.abs(previousVelocity)) {
            if (maxDeceleration / 50 < Math.abs(desiredVelocity - previousVelocity)) {
                return (desiredVelocity - previousVelocity > 0) ? 
                    previousVelocity + (maxDeceleration/50) : 
                    previousVelocity - (maxDeceleration/50);
            }
            else return desiredVelocity;
        }
        else if (Math.abs(desiredVelocity) > Math.abs(previousVelocity)) {
            if (maxAcceleration / 50 < Math.abs(desiredVelocity - previousVelocity)) {
                return (desiredVelocity - previousVelocity > 0) ?
                    previousVelocity + (maxAcceleration/50) :
                    previousVelocity - (maxAcceleration/50);
            }
            else return desiredVelocity;
        }
        else return desiredVelocity;
    }
}
