// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Limelight extends SubsystemBase {
  private static final double SINGLE_TAG_MAX_OMEGA_DEG_PER_SEC = 45.0;
  private static final double ROTATING_OMEGA_DEG_PER_SEC = 30.0;
  private static final double MAX_TRANSLATION_SPEED_FOR_VISION_MPS = 1.5;
  private static final double MAX_TRANSLATION_SPEED_FOR_SEED_MPS = 0.25;
  private static final double MAX_VISION_POS_ERROR_METERS = 1.0;
  private static final double AUTO_MAX_VISION_POS_ERROR_METERS = 0.5;

  CommandSwerveDrivetrain drivetrain;
  Alliance alliance;
  private String ll = VisionConstants.FRONT_LIMELIGHT;
  private Boolean enable = false;
  private Boolean trust = false;
  private int fieldError = 0;
  private int distanceError = 0;
  private boolean visionSeeded = false;
  private Pose2d botpose;
  private static final RectanglePoseArea field =
        new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));

  /** Creates a new Limelight. */
  public Limelight(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    SmartDashboard.putNumber("Field Error", fieldError);
    SmartDashboard.putNumber("Limelight Error", distanceError);
  }

@Override
public void periodic() {
    SmartDashboard.putBoolean("LL Periodic Running", true);
    SmartDashboard.putBoolean("LL Vision Seeded", visionSeeded);

    if (!enable) {
        SmartDashboard.putBoolean("LL Enabled", false);
        return;
    }
    SmartDashboard.putBoolean("LL Enabled", true);

    var driveState = drivetrain.getState();
    double gyroYawDeg = driveState.Pose.getRotation().getDegrees();
    double omegaDegPerSec = Math.toDegrees(driveState.Speeds.omegaRadiansPerSecond);
    double translationSpeedMps =
        Math.hypot(
            driveState.Speeds.vxMetersPerSecond,
            driveState.Speeds.vyMetersPerSecond);

    LimelightHelpers.SetRobotOrientation(ll, gyroYawDeg, omegaDegPerSec, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);
    //LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);
    SmartDashboard.putBoolean("LL Has Estimate", estimate != null);

    if (estimate == null) return;

    SmartDashboard.putNumber("LL Tag Count", estimate.tagCount);
    SmartDashboard.putNumber("LL Timestamp", estimate.timestampSeconds);
    SmartDashboard.putNumber("LL Avg Area", estimate.avgTagArea);
    SmartDashboard.putNumber("LL Vision X", estimate.pose.getX());
    SmartDashboard.putNumber("LL Vision Y", estimate.pose.getY());
    SmartDashboard.putNumber("LL Translation Speed MPS", translationSpeedMps);

    if (estimate.tagCount <= 0) return;
    if (estimate.timestampSeconds <= 0) return;
    final boolean isAuto = DriverStation.isAutonomousEnabled();
    if (isAuto && estimate.tagCount < 2) {
      SmartDashboard.putString("LL Reject Reason", "auto_requires_multi_tag");
      SmartDashboard.putBoolean("LL Added Vision", false);
      return;
    }
    if (estimate.tagCount == 1 && Math.abs(omegaDegPerSec) > SINGLE_TAG_MAX_OMEGA_DEG_PER_SEC) {
      SmartDashboard.putString("LL Reject Reason", "single_tag_high_omega");
      SmartDashboard.putBoolean("LL Added Vision", false);
      return;
    }
    if (translationSpeedMps > MAX_TRANSLATION_SPEED_FOR_VISION_MPS) {
      SmartDashboard.putString("LL Reject Reason", "high_translation_speed");
      SmartDashboard.putBoolean("LL Added Vision", false);
      return;
    }

    if (!field.isPoseWithinArea(estimate.pose)) {
        fieldError++;
        SmartDashboard.putNumber("Field Error", fieldError);
        SmartDashboard.putString("LL Reject Reason", "pose_off_field");
        SmartDashboard.putBoolean("LL Added Vision", false);
        return;
    }

    double posError = driveState.Pose.getTranslation()
        .getDistance(estimate.pose.getTranslation());

    SmartDashboard.putNumber("LL Pose Error", posError);
    if (!isAuto
        && !visionSeeded
        && estimate.tagCount >= 2
        && Math.abs(omegaDegPerSec) < ROTATING_OMEGA_DEG_PER_SEC
        && translationSpeedMps < MAX_TRANSLATION_SPEED_FOR_SEED_MPS) {
        // Bootstrap odometry from a high-confidence multi-tag solve.
        drivetrain.resetPose(estimate.pose);
        visionSeeded = true;
        SmartDashboard.putString("LL Reject Reason", "none_seeded_pose");
        SmartDashboard.putBoolean("LL Added Vision", true);
        return;
    }

    double maxVisionPosErrorMeters = isAuto ? AUTO_MAX_VISION_POS_ERROR_METERS : MAX_VISION_POS_ERROR_METERS;
    if (posError > maxVisionPosErrorMeters) {
        distanceError++;
        SmartDashboard.putNumber("Limelight Error", distanceError);
        SmartDashboard.putString("LL Reject Reason", "pos_error_gate");
        SmartDashboard.putBoolean("LL Added Vision", false);
        return;
    }

    if (Math.abs(omegaDegPerSec) > ROTATING_OMEGA_DEG_PER_SEC) {
        drivetrain.setVisionMeasurementStdDevs(
            VecBuilder.fill(1.5, 1.5, 60000000)
        );
    } else if (estimate.tagCount >= 2) {
        drivetrain.setVisionMeasurementStdDevs(
            VecBuilder.fill(0.4, 0.4, 60000000)
        );
    } else {
        drivetrain.setVisionMeasurementStdDevs(
            VecBuilder.fill(0.9, 0.9, 60000000)
        );
    }

    drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
    visionSeeded = true;
    SmartDashboard.putString("LL Reject Reason", "none");
    SmartDashboard.putBoolean("LL Added Vision", true);
}
  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  public void useLimelight(boolean enable) {
    this.enable = enable;
  }

  public void trustLL(boolean trust) {
    this.trust = trust;
  }
}