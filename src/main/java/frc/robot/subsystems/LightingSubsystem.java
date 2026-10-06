package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.util.Color.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.telemetry.Telemetry;

public class LightingSubsystem extends SubsystemBase {
  private SwerveSubsystem swerve;

  private static final LEDPattern rainbowLedPattern = LEDPattern.rainbow(255, 128);
  private static final Distance LED_SPACING = Units.Meters.of(1 / 120.0);
  private final LEDPattern scrollingRainbowLedPattern =
      rainbowLedPattern.scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(0.5), LED_SPACING);
  private static final LEDPattern yellowLEDPatern = LEDPattern.solid(Color.kYellow);
  private static final LEDPattern greenLedPattern = LEDPattern.solid(Color.kGreen);
  private static final LEDPattern whiteLedPattern = LEDPattern.solid(Color.kWhite);
  private static final LEDPattern purpleLedPattern = LEDPattern.solid(Color.kDarkViolet);
  private static final LEDPattern redLedPattern = LEDPattern.solid(kRed);
  private static final LEDPattern blueLedPattern = LEDPattern.solid(Color.kBlue);
  private static final LEDPattern orangeLedPattern = LEDPattern.solid(Color.kOrange);

  private static final AddressableLED ledStrip =
      new AddressableLED(Constants.LightingConstants.LED_STRING_PWM_PORT);
  private static final AddressableLEDBuffer ledBuffer =
      new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);

  public static Timer ledTimerOn = new Timer();
  public static Timer ledTimerOff = new Timer();

  public LightingSubsystem(SwerveSubsystem swerve) {
    this.swerve = swerve;

    ledStrip.setLength(Constants.LightingConstants.LED_STRING_LENGTH);
    ledStrip.start();

    ledTimerOn.reset();
    ledTimerOn.start();
    ledTimerOff.reset();
  }

  LEDPattern alliancePattern;

  @Override
  public void periodic() {

    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      alliancePattern = LEDPattern.solid(kRed);
    } else alliancePattern = LEDPattern.solid(kFirstBlue);

    if (DriverStation.isEnabled()) {
      if (Telemetry.drive.isBoosted) {
        scrollingRainbowLedPattern.applyTo(ledBuffer);
      } else {
        alliancePattern.applyTo(ledBuffer);
      }
    }

    ledStrip.setData(ledBuffer);
  }

  private void flashLed(AddressableLEDBuffer buffer, LEDPattern pattern) {
    if (ledTimerOn.get() >= .1) {
      ledTimerOn.reset();
      ledTimerOn.stop();
      ledTimerOff.start();

      pattern.applyTo(buffer);
      ledStrip.setData(buffer);
    } else if (ledTimerOff.get() >= .1) {
      ledTimerOff.reset();
      ledTimerOff.stop();
      ledTimerOn.start();

      LEDPattern.kOff.applyTo(buffer);
      ledStrip.setData(buffer);
    }
  }
}
