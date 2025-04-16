// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CorrectedColor;

public class LEDSubsystem extends SubsystemBase {
  private static final int kPort = 0;
  private static final int kLength = 270;

  private final Distance kLedSpacing = Meters.of(1 / 60.0);

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  private final int random = 6;//(int) Math.random() * 4;

  private boolean moving = false;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_led = new AddressableLED(kPort);
    m_ledBuffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();
  }

  @Override
  @SuppressWarnings("unused")
  public void periodic() {
    // This method will be called once per scheduler run
    if (!moving) {
      if (random == 0) {
        setPattern(LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(MetersPerSecond.of(2.0), kLedSpacing));
      }
      else if (random == 1) {
        setPattern(LEDPattern.solid(CorrectedColor.kRed).breathe(Seconds.of(2.0)));
      }
      else if (random == 2) {
        setPattern(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, CorrectedColor.kBlue, CorrectedColor.kDarkGreen, CorrectedColor.kGreen).scrollAtAbsoluteSpeed(MetersPerSecond.of(2.0), kLedSpacing));
      }
      else if (random == 3) {
        setPattern(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, CorrectedColor.kAqua, CorrectedColor.kBlack, CorrectedColor.kBlack, CorrectedColor.kAqua, CorrectedColor.kBlack, CorrectedColor.kBlack).scrollAtAbsoluteSpeed(MetersPerSecond.of(2), kLedSpacing));
      }
      else if (random == 4) {
        setPattern(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, CorrectedColor.kRed, CorrectedColor.kRed, CorrectedColor.kWhite, CorrectedColor.kBlue, CorrectedColor.kBlue).scrollAtAbsoluteSpeed(MetersPerSecond.of(-2), kLedSpacing));
      }
      else if (random == 5) {
        setPattern(LEDPattern.solid(CorrectedColor.kBlue));
      }
      else if (random == 6) {
        setPattern(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, CorrectedColor.kBlue, CorrectedColor.kBlack, CorrectedColor.kBlack, CorrectedColor.kBlue, CorrectedColor.kBlack, CorrectedColor.kBlack).scrollAtAbsoluteSpeed(MetersPerSecond.of(2), kLedSpacing));
      }
    }

    m_led.setData(m_ledBuffer);
  }

  public void setPattern(LEDPattern pattern) {
    pattern.applyTo(m_ledBuffer);
  }

  public void setMoving(boolean moving) {
    this.moving = moving;
  }
}
