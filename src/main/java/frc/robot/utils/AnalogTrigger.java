// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** 
 * A {@link Trigger} that gets its state from an axis on a controller.
 */
public class AnalogTrigger extends Trigger{
    /**
     * Creates an AnalogTrigger object to trigger commands.
     * 
     * @param controller Controller to use
     * @param axis Axis to watch
     * @param threshold Threshold before triggering
     */
    public AnalogTrigger(GenericHID controller, int axis, double threshold) {
        super(() -> controller.getRawAxis(axis) >= threshold);
        requireNonNullParam(controller, "controller", "AnalogTrigger");
    }
}
