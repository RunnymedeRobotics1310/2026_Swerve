package frc.robot.commands.operator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.auto.*;
import frc.robot.commands.swervedrive.SetAllianceGyroCommand;
import frc.robot.commands.test.SystemTestCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;
import java.util.function.Consumer;

/** The DriverController exposes all driver functions */
public class OperatorInput extends SubsystemBase {

  private final XboxController driverController;
  private final SwerveSubsystem swerve;
  private final LimelightVisionSubsystem vision;

  private Command autonomousCommand = new InstantCommand();

  private boolean matchNearEndTimerStarted = false;

  public enum RumblePattern {
    NONE(0, XboxController.RumbleType.kBothRumble, true),
    BLIP(0.25, XboxController.RumbleType.kBothRumble, true),
    SHORT(0.5, XboxController.RumbleType.kBothRumble, true),
    MEDIUM(1, XboxController.RumbleType.kBothRumble, true),
    RED_ALERT(2, XboxController.RumbleType.kBothRumble, true),
    TAG_ALIGN_LEFT(0.5, XboxController.RumbleType.kLeftRumble, false),
    TAG_ALIGN_RIGHT(0.5, XboxController.RumbleType.kRightRumble, false);

    public final double seconds;
    public final XboxController.RumbleType rumbleType;
    public final boolean driverController;

    RumblePattern(double seconds, XboxController.RumbleType rumbleType, boolean driverController) {
      this.seconds = seconds;
      this.rumbleType = rumbleType;
      this.driverController = driverController;
    }
  }

  private static RumblePattern currentRumblePattern = RumblePattern.NONE;
  private static final Timer rumbleTimer = new Timer();

  private final SendableChooser<Constants.AutoConstants.AutoPattern> autoPatternChooser =
      new SendableChooser<>();
  private final SendableChooser<Constants.AutoConstants.Delay> delayChooser =
      new SendableChooser<>();

  /**
   * Construct an OperatorInput class that is fed by a DriverController and an OperatorController.
   *
   * @param driverControllerPort on the driver station which the driver joystick is plugged into
   */
  public OperatorInput(
      int driverControllerPort,
      double deadband,
      SwerveSubsystem swerve,
      LimelightVisionSubsystem vision) {
    driverController = new GameController(driverControllerPort, deadband);
    this.swerve = swerve;
    this.vision = vision;
  }

  /**
   * Configure the button bindings for all operator commands
   *
   * <p>NOTE: This routine requires all subsystems to be passed in
   *
   * <p>NOTE: This routine must only be called once from the RobotContainer
   */
  public void configureButtonBindings(
      SwerveSubsystem driveSubsystem, LimelightVisionSubsystem visionSubsystem) {

    // System Test Command
    new Trigger(
            () ->
                driverController.getStartButton()
                    && driverController.getBackButton()
                    && !DriverStation.isFMSAttached())
        .onTrue(new SystemTestCommand(this, driveSubsystem));

    // Cancel Command
    new Trigger(this::isCancel).whileTrue(new CancelCommand(driveSubsystem));

    // Reset Gyro
    new Trigger(() -> driverController.getBackButton())
        .onTrue(new SetAllianceGyroCommand(driveSubsystem, 0));
  }

  /*
   * Cancel Command support
   * Do not end the command while the button is pressed
   */
  public boolean isCancel() {
    return (driverController.getStartButton() && !driverController.getBackButton());
  }

  public boolean isZeroGyro() {
    return driverController.getBackButton();
  }

  /*
   * Default Drive Command Buttons
   */
  public XboxController getRawDriverController() {
    return driverController;
  }

  public boolean getRotate180Val() {
    return driverController.getAButton();
  }

  /*
   * The following routines are used by the default commands for each subsystem
   *
   * They allow the default commands to get user input to manually move the
   * robot elements.
   */

  public boolean isFastMode() {
    return driverController.getRightBumperButton();
  }

  public boolean isSlowMode() {
    return driverController.getLeftBumperButton();
  }

  public boolean isFaceReef() {
    return driverController.getBButton();
  }

  public double getDriverControllerAxis(Stick stick, Axis axis) {
    switch (stick) {
      case LEFT:
        switch (axis) {
          case X:
            return driverController.getLeftX();
          case Y:
            return driverController.getLeftY();
        }
        break;
      case RIGHT:
        switch (axis) {
          case X:
            return driverController.getRightX();
        }
        break;
    }

    return 0;
  }

  // ALIGN CORAL STATION ANGLE
  public boolean isAlignLeftStation() {
    return driverController.getLeftTriggerAxis() > 0.5;
  }

  public boolean isAlignRightStation() {
    return driverController.getRightTriggerAxis() > 0.5;
  }

  /*
   * Support for haptic feedback to the driver
   */
  public void startVibrate() {
    driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
  }

  public void stopVibrate() {
    driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
  }

  @Override
  public void periodic() {
    if (Constants.TelemetryConfig.oi) {
      SmartDashboard.putString("Driver Controller", driverController.toString());
    }

    if (RobotState.isTeleop()
        && DriverStation.getMatchTime() > 0
        && DriverStation.getMatchTime() <= 20
        && !matchNearEndTimerStarted) {
      setRumblePattern(RumblePattern.SHORT);
      matchNearEndTimerStarted = true;
    }

    double matchTimeRemaining = RunnymedeUtils.teleopMatchTimeRemaining();
    if (matchTimeRemaining < 0.2 && matchTimeRemaining > 0) {
      setRumblePattern(RumblePattern.BLIP);
    }

    rumbleUpdate();
  }

  public enum Stick {
    LEFT,
    RIGHT
  }

  public enum Axis {
    X,
    Y
  }

  public static void setRumblePattern(RumblePattern pattern) {
    synchronized (rumbleTimer) {
      if (pattern != currentRumblePattern) {
        currentRumblePattern = pattern;
        rumbleTimer.restart();
      }
    }
  }

  private void rumbleUpdate() {
    synchronized (rumbleTimer) {
      if (!rumbleTimer.isRunning()) {
        return;
      }

      double time = rumbleTimer.get();
      double rumbleAmount = 1;

      // stop after rumble duration seconds
      if (time > currentRumblePattern.seconds) {
        currentRumblePattern = RumblePattern.NONE;
        rumbleTimer.stop();
        rumbleAmount = 0.0;
      }

      if (currentRumblePattern.driverController) {
        driverController.setRumble(currentRumblePattern.rumbleType, rumbleAmount);
      }
    }
  }

  public void initAutoSelectors() {

    SmartDashboard.putData("1310/auto/Auto Selector", autoPatternChooser);

    autoPatternChooser.setDefaultOption(
        "Do Nothing", Constants.AutoConstants.AutoPattern.DO_NOTHING);
    autoPatternChooser.addOption("Exit Zone", Constants.AutoConstants.AutoPattern.EXIT_ZONE);

    autoPatternChooser.onChange(
        new Consumer<Constants.AutoConstants.AutoPattern>() {
          @Override
          public void accept(Constants.AutoConstants.AutoPattern pattern) {
            autonomousCommand = generateAutonomousCommand(delayChooser.getSelected(), pattern);
          }
        });

    SmartDashboard.putData("1310/auto/Delay Selector", delayChooser);

    delayChooser.setDefaultOption("No Delay", Constants.AutoConstants.Delay.NO_DELAY);
    delayChooser.addOption("1/2 Seconds", Constants.AutoConstants.Delay.WAIT_0_5_SECOND);
    delayChooser.addOption("1 Second", Constants.AutoConstants.Delay.WAIT_1_SECOND);
    delayChooser.addOption("1 1/2 Seconds", Constants.AutoConstants.Delay.WAIT_1_5_SECONDS);
    delayChooser.addOption("2 Seconds", Constants.AutoConstants.Delay.WAIT_2_SECONDS);
    delayChooser.addOption("2 1/2 Seconds", Constants.AutoConstants.Delay.WAIT_2_5_SECONDS);
    delayChooser.addOption("3 Seconds", Constants.AutoConstants.Delay.WAIT_3_SECONDS);
    delayChooser.addOption("5 Seconds", Constants.AutoConstants.Delay.WAIT_5_SECONDS);

    delayChooser.onChange(
        new Consumer<Constants.AutoConstants.Delay>() {
          @Override
          public void accept(Constants.AutoConstants.Delay delay) {
            autonomousCommand = generateAutonomousCommand(delay, autoPatternChooser.getSelected());
          }
        });

    // Set default autonomous command
    autonomousCommand =
        generateAutonomousCommand(delayChooser.getSelected(), autoPatternChooser.getSelected());
  }

  private Command generateAutonomousCommand(
      Constants.AutoConstants.Delay delayChoice,
      Constants.AutoConstants.AutoPattern patternChoice) {

    double delay =
        switch (delayChoice) {
          case WAIT_0_5_SECOND -> 0.5;
          case WAIT_1_SECOND -> 1;
          case WAIT_1_5_SECONDS -> 1.5;
          case WAIT_2_SECONDS -> 2;
          case WAIT_2_5_SECONDS -> 2.5;
          case WAIT_3_SECONDS -> 3;
          case WAIT_5_SECONDS -> 5;
          default -> 0;
        };

    return switch (patternChoice) {
      case EXIT_ZONE -> new ExitZoneAutoCommand(swerve, delay);

      default -> new InstantCommand();
    };
  }

  public Command getAutonomousCommand() {
    return autonomousCommand;
  }
}
