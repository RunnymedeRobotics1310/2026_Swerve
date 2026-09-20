package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class LightingSubsystem extends SubsystemBase {
  private SwerveSubsystem swerve;

  //  private static

  public LightingSubsystem(SwerveSubsystem swerve) {
    this.swerve = swerve;
  }
}
