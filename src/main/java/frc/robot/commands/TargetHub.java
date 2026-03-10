// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TagLists;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetHub extends Command {
  /** Creates a new TargetHub. */
    private DoubleSupplier m_X;
    private DoubleSupplier m_Y; 
    private  CommandSwerveDrivetrain m_drivetrain; 
    private VisionSubsystem m_vision;
    private List<Integer> tags;
    private final SwerveRequest.RobotCentric drive = 
    new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private double rot_kP = .0075;
    private double vision_desired_angle = 0.0;
    private double max_rot_speed = DriveConstants.MaxAngularRate;

  public TargetHub(DoubleSupplier driverX, DoubleSupplier driverY, CommandSwerveDrivetrain drive, VisionSubsystem vision) {
    m_X = driverX;
    m_Y = driverY;
    m_drivetrain = drive;
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
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
 
    if (tags.contains(m_vision.getTagRawInt())){
      targetYaw = LimelightHelpers.getTX("limelight");
      visibleTarget = true;
    }
    if (visibleTarget){
      turn1 = (vision_desired_angle + targetYaw) * rot_kP *max_rot_speed;
      turn2 = MathUtil.applyDeadband(turn1,0.1);
      turn3 = MathUtil.applyDeadband(MathUtil.interpolate(turn1,turn2,0.5),0.1,0.65);
      SmartDashboard.putNumber("turnout1", turn1);
      SmartDashboard.putNumber("turnout2", turn2);
      SmartDashboard.putNumber("turnout3", turn3);
    }
    m_drivetrain.setControl(
      drive.withVelocityX(-m_X.getAsDouble())
      .withVelocityY(-m_Y.getAsDouble())
      .withRotationalRate(turn3)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_drivetrain.setControl(
      drive.withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
