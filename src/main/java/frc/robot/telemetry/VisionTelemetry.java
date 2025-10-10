/*
 * Copyright 2025 The Kingsway Digital Company Limited. All rights reserved.
 */
package frc.robot.telemetry;

import static frc.robot.telemetry.Telemetry.PREFIX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.VisionTelemetryLevel;

/**
 * @author JZ
 * @since 2025-02-16 10:45
 */
public class VisionTelemetry {

  /** Whether the vision telemetry is enabled */
  public VisionTelemetryLevel telemetryLevel = VisionTelemetryLevel.NONE;

  // Pose
  /** The x location of the robot with respect to the field in metres */
  public double poseMetresX = -1;

  /** The y location of the robot with respect to the field in metres */
  public double poseMetresY = -1;

  /** The heading of the robot with respect to the field in degrees */
  public double poseHeadingDegrees = -1310;

  /**
   * The x location of the robot with respect to the field as measured by the vision system in
   * metres
   */
  public double visionPoseX = -1;

  /**
   * The y location of the robot with respect to the field as measured by the vision system in
   * metres
   */
  public double visionPoseY = -1;

  /**
   * The heading of the robot with respect to the field as measured by the vision system in degrees
   */
  public double visionPoseHeading = -1310;

  /** The distance offset of vision pose to odometry */
  public double poseDeltaMetres = -1310;

  /** The heading offset of vision pose to odometry */
  public double headingDeltaDegrees = -1310;

  /** The tag ambiguity of the currently in focus tag */
  public double tagAmbiguity = -1;

  public double navxYaw = -1310;

  public double navxYawDelta = -1310;

  public TimeSeriesMetric poseXSeries = new TimeSeriesMetric();
  public TimeSeriesMetric poseYSeries = new TimeSeriesMetric();
  public TimeSeriesMetric poseDegSeries = new TimeSeriesMetric();

  public double[] nikVisibleTags = new double[0];
  public double nikTx = -1;
  public double nikDistanceToRobot = -1;
  public double nikDistanceToCam = -1;

  public double[] tomVisibleTags = new double[0];
  public double tomTx = -1;
  public double tomDistanceToRobot = -1;
  public double tomDistanceToCam = -1;

  public void addPoseToSeries(Pose2d pose) {
    poseXSeries.add(pose.getTranslation().getX());
    poseYSeries.add(pose.getTranslation().getY());
    poseDegSeries.add(pose.getRotation().getDegrees());
  }

  void post() {

    if (Telemetry.vision.telemetryLevel == VisionTelemetryLevel.REGULAR
        || Telemetry.vision.telemetryLevel == VisionTelemetryLevel.VERBOSE) {
      String poseOdo =
          String.format("(%.2f, %.2f)m %.1f°", poseMetresX, poseMetresY, poseHeadingDegrees);
      SmartDashboard.putString(PREFIX + "Vision/pose_odo", poseOdo);

      String poseVis =
          String.format("(%.2f, %.2f)m %.1f°", visionPoseX, visionPoseY, visionPoseHeading);
      SmartDashboard.putString(PREFIX + "Vision/pose_vis", poseVis);

      String poseDelta = String.format("%.2fm, %.1f°", poseDeltaMetres, headingDeltaDegrees);
      SmartDashboard.putString(PREFIX + "Vision/pose_delta", poseDelta);

      SmartDashboard.putNumber(PREFIX + "Vision/tag_ambiguity", tagAmbiguity);

      SmartDashboard.putString(PREFIX + "Vision/yaw_navx", String.format("%.2f°", navxYaw));

      SmartDashboard.putString(
          PREFIX + "Vision/yaw_navx_delta", String.format("%.2f°", navxYawDelta));
    }

    if (Telemetry.vision.telemetryLevel == VisionTelemetryLevel.VERBOSE) {
      String poseVisSeries =
          String.format(
              "(%.2f, %.2f)m %.1f°",
              poseXSeries.getMaxValue() - poseXSeries.getMinValue(),
              poseYSeries.getMaxValue() - poseYSeries.getMinValue(),
              poseDegSeries.getMaxValue() - poseDegSeries.getMinValue());
      SmartDashboard.putString(PREFIX + "Vision/pose_vis_range", poseVisSeries);

      SmartDashboard.putNumberArray(PREFIX + "Vision/nik_visible_tags", nikVisibleTags);
      SmartDashboard.putNumber(PREFIX + "Vision/nik_tx", nikTx);
      SmartDashboard.putNumber(PREFIX + "Vision/nik_distanceToRobot", nikDistanceToRobot);
      SmartDashboard.putNumber(PREFIX + "Vision/nik_distanceToCam", nikDistanceToCam);

      SmartDashboard.putNumberArray(PREFIX + "Vision/tom_visible_tags", tomVisibleTags);
      SmartDashboard.putNumber(PREFIX + "Vision/tom_tx", tomTx);
      SmartDashboard.putNumber(PREFIX + "Vision/tom_distanceToRobot", tomDistanceToRobot);
      SmartDashboard.putNumber(PREFIX + "Vision/tom_distanceToCam", tomDistanceToCam);
    }
  }
}
