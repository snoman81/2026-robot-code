// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    // ------------Define Motors------------------
  public final TalonFX MainMotor = new TalonFX(ShooterConstants.kMainMotorPort, Constants.rioBus);
  public final TalonFX FollowMotor = new TalonFX(ShooterConstants.kFollowMotorPort, Constants.rioBus);
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    SetShooterConfigs();
  }

  // -----cfgs-------------------------------------------------------------
  public void SetShooterConfigs(){
    var motorConfigurator1 = MainMotor.getConfigurator();
    var motorConfigurator2 = FollowMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();  
    motorConfigs
    .withMotorOutput(
    new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Coast)
    .withInverted(InvertedValue.valueOf("CounterClockwise_Positive"))
    )
    .withCurrentLimits(
    new CurrentLimitsConfigs()
    .withStatorCurrentLimitEnable(ShooterConstants.kMotorCurrentLimitEnable)
    .withStatorCurrentLimit(ShooterConstants.kMotorCurrentLimit)
    );
    // slot0 velocity configs
    var slot0configs = new Slot0Configs();
    slot0configs.kS = ShooterConstants.kS;
    slot0configs.kV = ShooterConstants.kV;
    slot0configs.kA = ShooterConstants.kA;
    slot0configs.kP = ShooterConstants.kP;
    slot0configs.kI = ShooterConstants.kI;
    slot0configs.kD = ShooterConstants.kD;
    // Config main MOTOR FIRST
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motorConfigurator1.apply(motorConfigs);
      status = motorConfigurator1.apply(slot0configs);
      status = motorConfigurator2.apply(motorConfigs);
      status = motorConfigurator2.apply(slot0configs);
      if (status.isOK()) break;
    }
    if (!status.isOK() ) {
      System.out.println("Could not apply configs, error code Status 1: " + status.toString());
    }
    /** dont have to do this??
    //Invert and config Follow motor
    motorConfigs.withMotorOutput(
      new MotorOutputConfigs()
      .withInverted(InvertedValue.valueOf("Clockwise_Postitive"))
    );
    
        /* Retry config apply up to 5 times, report if failure 
    for (int i = 0; i < 5; ++i) {
      status = motorConfigurator2.apply(motorConfigs);
      if (status.isOK()) break;
    }
    if (!status.isOK() ) {
      System.out.println("Could not apply configs, error code Status 1: " + status.toString());
    }
    */
    //Set Follow to Follow
    FollowMotor.setControl(new Follower(MainMotor.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  public void SetVelocity(int rps){
    final VelocityTorqueCurrentFOC m_request = new VelocityTorqueCurrentFOC(0)
    .withSlot(0);

    MainMotor.setControl(m_request.withVelocity(rps));
  }
  public double GetVelocity(){
    return MainMotor.getVelocity().getValueAsDouble();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", GetVelocity());
  
  }
}
