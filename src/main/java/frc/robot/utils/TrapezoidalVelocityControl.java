// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * Helper class that allows velocity to be calculated/set in "steps," which allows for accceleration and decceleration
 * control and allows for smoother motion. It will output in the same units that are inputted.
 */
public class TrapezoidalVelocityControl {
    private final double maxDeceleration;
    private final double maxAcceleration;

    private final double controlFramesPerSecond;

    /** Creates a new TrapezoidalVelocityControl profile. 
     * 
     * @param maxDeceleration Max deceleration in units per second
     * @param maxAcceleration Max acceleration in units per second
     */
    public TrapezoidalVelocityControl(double maxDeceleration, double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.controlFramesPerSecond = 50;
    }

    /** Creates a new TrapezoidalVelocityControl profile. 
     * 
     * @param maxDeceleration Max deceleration in units per second
     * @param maxAcceleration Max acceleration in units per second
     * @param controlPeriod Interval at which robot code runs
     */
    public TrapezoidalVelocityControl(double maxDeceleration, double maxAcceleration, double controlPeriod) {
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.controlFramesPerSecond = 1/controlPeriod;
    }

    /**
     * Calcuates next setpoint based on desired velocity and current velocity.
     * 
     * @param desiredVelocity Desired velocity
     * @param currentVelocity Current velocity
     * @return Next setpoint
     */
    public double getSetpoint(double desiredVelocity, double currentVelocity) {
        //if slowing down
        if (Math.abs(desiredVelocity) < Math.abs(currentVelocity)) {
            //returns the next "step" if difference between desired and current velocity is too big
            if (maxDeceleration / controlFramesPerSecond < Math.abs(desiredVelocity - currentVelocity)) {
                return (desiredVelocity - currentVelocity > 0) ? 
                    currentVelocity + (maxDeceleration/controlFramesPerSecond) : 
                    currentVelocity - (maxDeceleration/controlFramesPerSecond);
            }

            //returns the desired velocity as a setpoint if difference between desired and current velocity is small enough
            else return desiredVelocity;
        }
        
        //if speeding up
        else if (Math.abs(desiredVelocity) > Math.abs(currentVelocity)) {
            //returns the next "step" if difference between desired and current velocity is too big
            if (maxAcceleration / 50 < Math.abs(desiredVelocity - currentVelocity)) {
                return (desiredVelocity - currentVelocity > 0) ?
                    currentVelocity + (maxAcceleration/controlFramesPerSecond) :
                    currentVelocity - (maxAcceleration/controlFramesPerSecond);
            }
            else return desiredVelocity;
        }

        //returns the desired velocity as a setpoint if difference between desired and current velocity is small enough
        else return desiredVelocity;
    }
}
