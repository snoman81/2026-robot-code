// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.KickerConstants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.KickerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HopperRun extends Command {
  /** Creates a new HopperRun. */
  private HopperSubsystem m_hopper;
  private KickerSubsystem m_kicker;
  public HopperRun(HopperSubsystem hopper, KickerSubsystem kicker) {
    m_hopper = hopper;
    m_kicker = kicker;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hopper);
    addRequirements(m_kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hopper.setDutyCycleOut(HopperConstants.m_HopperSpeed);
    m_kicker.setVelocity(KickerConstants.m_KickerVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.setDutyCycleOut(0);
    m_kicker.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
