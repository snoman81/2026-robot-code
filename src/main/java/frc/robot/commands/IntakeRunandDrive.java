// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeRunandDrive extends Command {
  private DoubleSupplier m_X;
  private DoubleSupplier m_Y;
  private DoubleSupplier m_Omega;
  private final SwerveRequest.RobotCentric drive = 
  new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private IntakeSubsystem m_intake;
  private CommandSwerveDrivetrain m_drivetrain;
  private double speed = 0;
  /** Creates a new IntakeRun. */
  public IntakeRunandDrive(DoubleSupplier driverX, DoubleSupplier driverY,DoubleSupplier driverROT,IntakeSubsystem intake, CommandSwerveDrivetrain drive) {
    m_X = driverX;
    m_Y = driverY;
    m_Omega = driverROT;
    m_intake = intake;
    m_drivetrain = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
    addRequirements(m_drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.setControl(
      drive.withVelocityX(-m_X.getAsDouble()*IntakeConstants.m_IntakeDriveSpeedMax)
           .withVelocityY(-m_Y.getAsDouble()*IntakeConstants.m_IntakeDriveSpeedMax)
           .withRotationalRate(m_Omega.getAsDouble()));
           
    double chassisspeed = m_drivetrain.ChassisSpeedsCalc();
    speed = m_intake.getRPM(chassisspeed);
    m_intake.setRollerSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setRollerNeutral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
