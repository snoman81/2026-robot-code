// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterRange extends Command {
  /** Creates a new ShooterRange. */
  private ShooterSubsystem m_shooter;
  AprilTagFieldLayout layout =
    AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private List<Integer> allowed_tags = Arrays.asList(1,2,3);

  public ShooterRange(ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
    shooter = m_shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tagg = LimelightHelpers.getFiducialID("limelight");
    int tag = (int) tagg;
    var PoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limglight");
    if (PoseEstimate != null && PoseEstimate.tagCount >= 2){
      Pose2d robotPose = PoseEstimate.pose;
      double distance = robotPose.getTranslation()
      .getDistance(layout.getTagPose(tag).get().toPose2d().getTranslation());
    }



    double shooterspeed = 0;
    boolean visibleTarget = false;

    if (allowed_tags.contains(LimelightHelpers.getFiducialID("limelight"))){

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
