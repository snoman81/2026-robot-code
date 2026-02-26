// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetHub extends Command {
  /** Creates a new TargetHub. */
    private  CommandSwerveDrivetrain m_drivetrain; 

    private double rot_kP = .035;
    private double vision_desired_angle = 0.0;
    private double max_rot_speed = DriveConstants.MaxAngularRate;
    private List<Integer> allowed_tags = Arrays.asList(1,2,3);

    private final SwerveRequest.RobotCentric drive = 
    new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public TargetHub(CommandSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    drive = m_drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    double turn = 0.0;
    double targetYaw = 0.0;
    boolean visibleTarget = false;
    
    if (allowed_tags.contains(LimelightHelpers.getFiducialID("limelight"))){
      targetYaw = LimelightHelpers.getTX("limelight");
      visibleTarget = true;
    }
    if (visibleTarget){
      turn = (vision_desired_angle - targetYaw) *rot_kP *max_rot_speed;
    }
    m_drivetrain.setControl(
      drive.withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(turn)
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
