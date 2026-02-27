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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TagLists;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterRange extends Command {
  /** Creates a new ShooterRange. */
  private ShooterSubsystem m_shooter;
  private VisionSubsystem m_vision;
  private AprilTagFieldLayout layout =
    AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private List<Integer> tags;

  private double distance_to_goal = 0.0;

  public ShooterRange(ShooterSubsystem shooter, VisionSubsystem vision) {
    m_shooter = shooter;
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
    addRequirements(m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Alliance alliance =
    DriverStation.getAlliance().orElse(Alliance.Blue);

    if (alliance == Alliance.Red) {
           tags = TagLists.redTags;
        }
    else{tags = TagLists.blueTags;}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterspeed = 0;
    boolean visibleTarget = false;
    var PoseEstimate = m_vision.getEstimatedGlobalPose();
   
    if (PoseEstimate != null && m_vision.getTagRawInt() >= 2){
      Pose2d robotPose = PoseEstimate.get();
      double distance_to_goal = robotPose.getTranslation()
      .getDistance(layout.getTagPose(m_vision.getTagRawInt()).get().toPose2d().getTranslation());
    }

    if (tags.contains(m_vision.getTagRawInt())){
      visibleTarget = true;
      shooterspeed = m_shooter.getRPM(distance_to_goal);
    }
    if (visibleTarget){
      m_shooter.SetVelocity(shooterspeed);
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
