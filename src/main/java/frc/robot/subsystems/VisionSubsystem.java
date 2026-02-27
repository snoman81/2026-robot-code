// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(){}

   // ========================
    // CONFIGURATION
    // ========================

    private static final String LIMELIGHT = "limelight";

    // Camera position relative to robot center
    private static final Transform2d CAMERA_OFFSET =
        new Transform2d(
            new Translation2d(0.0, 0.0),   // I set it in dashboard but set it here
            new Rotation2d()
        );

    // Blue alliance target pose (hub location)
    private static final Pose2d BLUE_TARGET_POSE =
        new Pose2d(4.620, 4.04, new Rotation2d());

    // Filtering
    private final LinearFilter distanceFilter =
        LinearFilter.singlePoleIIR(0.2, 0.02);

    // Cached values
    private Pose2d lastGoodPose = new Pose2d();
    private double confidenceScore = 0.0;
    // ========================
    // PUBLIC API
    // ========================
    /**
     * Returns filtered, camera-offset corrected field pose from MegaTag2
     */
    public Optional<Pose2d> getEstimatedGlobalPose() {

        var estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT);

        if (!isValidEstimate(estimate)) {
            return Optional.empty();
        }

        Pose2d pose = estimate.pose;

        // Apply camera offset correction if needed
        pose = pose.transformBy(
            new Transform2d(
                CAMERA_OFFSET.getTranslation().unaryMinus(),
                new Rotation2d()
            )
        );

        lastGoodPose = pose;
        confidenceScore = calculateConfidence(estimate);

        return Optional.of(pose);
    }

    /**
     * Distance from robot to scoring target
     */
    public Optional<Double> getDistanceToTarget() {

        return getEstimatedGlobalPose().map(pose -> {
            double raw =
                pose.getTranslation()
                    .getDistance(getTargetPose().getTranslation());

            return distanceFilter.calculate(raw);
        });
    }

    /**
     * Desired chassis heading to face target
     */
    public Optional<Rotation2d> getTargetHeading() {

        return getEstimatedGlobalPose().map(pose -> {

            Translation2d delta =
                getTargetPose().getTranslation()
                    .minus(pose.getTranslation());

            return new Rotation2d(
                Math.atan2(delta.getY(), delta.getX())
            );
        });
    }

    public boolean isPoseReliable() {
        return confidenceScore > 0.5;
    }

    public double getConfidenceScore() {
        return confidenceScore;
    }

    public int getTagRawInt(){
      double tagdouble = LimelightHelpers.getFiducialID(LIMELIGHT);
      Integer tagint = (int) tagdouble;
      return tagint;
    }

    public int tagCount(){
      return LimelightHelpers.getTargetCount(LIMELIGHT);
    }

    /* dunno if i need them to be optional sooo ill leave these here
    public Optional<Integer> getTagRawInt(){
      double tagdouble = LimelightHelpers.getFiducialID(LIMELIGHT);
      Integer tagint = (int) tagdouble;
      return Optional.of(tagint);
    }

    public Optional<Integer> tagCount(){
      return Optional.of(LimelightHelpers.getTargetCount(LIMELIGHT));
    }
 */



    // ========================
    // INTERNAL METHODS
    // ========================

    private boolean isValidEstimate(
        LimelightHelpers.PoseEstimate estimate
    ) {
        if (estimate == null) return false;
        if (estimate.tagCount < 2) return false;
        if (estimate.avgTagArea < 0.1) return false;
        if (estimate.pose == null) return false;

        return true;
    }

    private double calculateConfidence(
        LimelightHelpers.PoseEstimate estimate
    ) {
        double tagWeight = Math.min(estimate.tagCount / 3.0, 1.0);
        double areaWeight = Math.min(estimate.avgTagArea / 0.4, 1.0);
        double ambiguityWeight =
            1.0 - Math.min(estimate.avgTagDist / 6.0, 1.0);

        return (tagWeight * 0.4)
             + (areaWeight * 0.3)
             + (ambiguityWeight * 0.3);
    }

    private Pose2d getTargetPose() {

        Alliance alliance =
            DriverStation.getAlliance().orElse(Alliance.Blue);

        if (alliance == Alliance.Red) {
            return new Pose2d(
              16.5405 - BLUE_TARGET_POSE.getX(),
              BLUE_TARGET_POSE.getY(),
              BLUE_TARGET_POSE.getRotation().rotateBy(Rotation2d.k180deg)
            );
        }

        return BLUE_TARGET_POSE;
    }


     
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SignalLogger.writeString("LL Name", LIMELIGHT);
    getEstimatedGlobalPose().ifPresent(pose -> 
    SignalLogger.writeDouble("pose2dx", pose.getX()));
    getEstimatedGlobalPose().ifPresent(pose -> 
    SignalLogger.writeDouble("pose2dy", pose.getY()));
  }
}
