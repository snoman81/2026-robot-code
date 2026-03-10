// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.lang.StackWalker.Option;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TagLists;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetHubandShootRange extends Command {
  /** Creates a new TargetHub. */
    private DoubleSupplier m_X;
    private DoubleSupplier m_Y;
    private DoubleSupplier m_Omega;
    private ShooterSubsystem m_shooter;
    private  CommandSwerveDrivetrain m_drivetrain; 
    private VisionSubsystem m_vision;
    private List<Integer> tags;
    private AprilTagFieldLayout layout =
    AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final SwerveRequest.RobotCentric drive = 
    new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private double distance_to_goal = 0.0;
    private double shooterspeed = 0;
    private boolean visibleTarget = false;
    private double rot_kP = .0075;
    private double vision_desired_angle = 0.0;
    private double max_rot_speed = DriveConstants.MaxAngularRate;

  public TargetHubandShootRange(DoubleSupplier driverX, DoubleSupplier driverY,DoubleSupplier driverROT, CommandSwerveDrivetrain drive, ShooterSubsystem shooter, VisionSubsystem vision) {
    m_X = driverX;
    m_Y = driverY;
    m_Omega = driverROT;
    m_shooter = shooter;
    m_drivetrain = drive;
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
    addRequirements(m_drivetrain);
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
    else{tags = TagLists.blueTags;} //i dont trust it to work without else :sob:
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    double turn1 = 0.0;
    double turn2 = 0.0;
    double turn3 = 0.0;
    double targetYaw = 0.0;
    boolean visibleTarget = false;
    boolean inrange = false;
    var PoseEstimate = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
    Optional<Pose3d> pose = layout.getTagPose(m_vision.getTagRawInt());
    SmartDashboard.putNumber("tagcount", m_vision.tagCount());
    SmartDashboard.putNumber("tagID", m_vision.getTagRawInt());
    SmartDashboard.putBoolean("right tag?", tags.contains(m_vision.getTagRawInt()));
    if (pose.isPresent() && m_vision.tagCount() >= 2){
      //Pose2d robotPose = PoseEstimate.get();
      distance_to_goal = PoseEstimate.getTranslation()
      .getDistance(layout.getTagPose(m_vision.getTagRawInt()).get().toPose2d().getTranslation());
      inrange = (distance_to_goal >= 1.25);
      SmartDashboard.putNumber("dist?", distance_to_goal);
      SmartDashboard.putBoolean("inrange?", inrange);
        
    if (tags.contains(m_vision.getTagRawInt())){
      shooterspeed = m_shooter.getRPM(distance_to_goal);
      targetYaw = LimelightHelpers.getTX("limelight");
      visibleTarget = true;
      SmartDashboard.putBoolean("target?", visibleTarget);
      SmartDashboard.putNumber("FetchedRPM", shooterspeed);

    if (visibleTarget){
      turn1 = (vision_desired_angle + targetYaw) * rot_kP *max_rot_speed;
      turn2 = MathUtil.applyDeadband(turn1,0.1);
      turn3 = MathUtil.applyDeadband(MathUtil.interpolate(turn1,turn2,0.5),0.1,0.65);
      SmartDashboard.putNumber("turnout1", turn1);
      SmartDashboard.putNumber("turnout2", turn2);
      SmartDashboard.putNumber("turnout3", turn3);
      m_drivetrain.setControl(
        drive.withVelocityX(-m_Y.getAsDouble())
        .withVelocityY(-m_X.getAsDouble())
        .withRotationalRate(turn2)
        );
    if (inrange){
      m_shooter.SetVelocity(shooterspeed);
    }
    }
    }
    }
        else {
            m_drivetrain.setControl(
        drive.withVelocityX(-m_X.getAsDouble())
        .withVelocityY(-m_Y.getAsDouble())
        .withRotationalRate(m_Omega.getAsDouble()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_drivetrain.setControl(
      drive.withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(0));
      m_shooter.setNeutral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
