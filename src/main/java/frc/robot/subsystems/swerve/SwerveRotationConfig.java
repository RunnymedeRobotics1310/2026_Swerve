package frc.robot.subsystems.swerve;

public record SwerveRotationConfig(
    double maxRotVelocityDegPS,
    double defaultRotVelocityDegPS,
    double maxAccelerationDegPS2,
    double headingP,
    double headingI,
    double headingD) {}
