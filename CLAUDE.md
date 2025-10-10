
# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is FRC Team 1310 (Runnymede Robotics) 2026 swerve drive robot code. The project uses WPILib's command-based framework and depends on a custom Runnymede Swerve Library (`ca.team1310:swerve`) published to Maven Central.

## Build & Development Commands

### Building
```bash
./gradlew build
```

### Deploying to Robot
```bash
./gradlew deploy
```

### Running Tests
```bash
./gradlew test
```

### Simulation
```bash
./gradlew simulateJava
```

### Clean Build (Required when updating RunnymedeSwerve library)
```bash
./gradlew clean && ./gradlew compileJava
```

### Single Test
```bash
./gradlew test --tests ClassName.testMethodName
```

## Architecture

### Command-Based Structure
The codebase follows WPILib's command-based paradigm:
- **Robot.java**: Main robot class, handles mode transitions, port forwarding for Limelights, and telemetry updates (every 150ms)
- **RobotContainer.java**: Initializes subsystems, sets default commands, configures button bindings, and provides autonomous command
- **Constants.java**: Centralized configuration including swerve module configs, PID values, field geometry, and telemetry flags

### Subsystems
1. **SwerveSubsystem** (`subsystems/swerve/`): Wraps `RunnymedeSwerveDrive` from the external library. Provides field-oriented and robot-oriented drive, rate limiting via `SlewRateLimiter`, heading PID control, and convenience methods like `computeOmega()` and `getClosestReefAngle()`.

2. **LimelightVisionSubsystem** (`subsystems/vision/`): Manages Limelight vision processing, integrates with swerve odometry via `LimelightAwareSwerveDrive`.

### Commands
- **TeleopDriveCommand**: Default drive command that reads from `OperatorInput`, supports field-oriented drive, speed modes (fast/slow/normal), and rotation behaviors (manual, 180°, face reef, align to stations)
- **Auto Commands** (`commands/auto/`): Autonomous routines like `ExitZoneAutoCommand`
- **Test Commands** (`commands/test/SystemTestCommand`): System testing triggered by Start+Back buttons when not on FMS

### Operator Input
**OperatorInput.java** manages Xbox controller input with deadband handling, button bindings, rumble patterns (match timing, haptic feedback), and autonomous mode selection via SmartDashboard choosers.

### Configuration Pattern
Subsystem configs are immutable records:
- `SwerveDriveSubsystemConfig`: Aggregates core swerve, gyro, limelight, translation, and rotation configs
- `SwerveTranslationConfig`: Max speeds, acceleration limits, velocity PID
- `SwerveRotationConfig`: Rotation velocity limits, heading PID

### Hardware Configuration
- **Gyro**: Pigeon 2 (CAN ID 8)
- **Swerve Modules**: SDS MK4i with NEO/NEO 550 motors, CANCoder absolute encoders
  - Module CAN IDs: FrontLeft (10-12), FrontRight (20-22), BackRight (30-32), BackLeft (40-42)
- **Limelights**: "nikola" (primary at 10.13.10.11), "thomas" (secondary at 10.13.10.12)

### Telemetry
Custom telemetry system (`telemetry/Telemetry.java`) with per-subsystem enable flags in `Constants.TelemetryConfig`. Posted every 150ms in `Robot.robotPeriodic()`.

## Code Formatting

This project uses Google Java Format. Format code before committing. IntelliJ plugin requires specific VM options (see README.md).

## Dependencies

- **WPILib**: Standard FRC library (2025 season)
- **RunnymedeSwerve Library**: `ca.team1310:swerve:3.3.5` (Maven Central or local maven)
- **Vendor Libraries**: Phoenix 6 (CTRE), REVLib, Studica
- **Testing**: JUnit 5

When updating the RunnymedeSwerve library version in `build.gradle`, run `./gradlew clean && ./gradlew compileJava` to fetch the new version.

## Field Coordinates

Uses 2025 FRC field with AprilTag locations defined in `Constants.FieldConstants.TAGS`. Coordinate system: positive X away from blue alliance wall, positive Y to the left. Field dimensions: 17.55m x 8.052m.
