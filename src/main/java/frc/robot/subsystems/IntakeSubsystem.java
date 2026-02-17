// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  // ------------Define Motors------------------
  public final TalonFX RollerMotor = new TalonFX(IntakeConstants.kRollerMotorPort, Constants.rioBus);
  public final TalonFX PivotMotor = new TalonFX(IntakeConstants.kIntakePivotMotorPort, Constants.rioBus);
  /** Creates a new Intake. */
  public IntakeSubsystem() {
    SetRollerConfig();
    SetPivotConfig();
  }

  
// -----cfgs-------------------------------------------------------------
  public void SetRollerConfig(){
    var motorConfigurator = RollerMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();  
    motorConfigs
      .withMotorOutput(
      new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Coast)
    )
    .withCurrentLimits(
      new CurrentLimitsConfigs()
      .withStatorCurrentLimitEnable(IntakeConstants.kRollerMotorCurrentLimitEnable)
      .withStatorCurrentLimit(IntakeConstants.kRollerMotorCurrentLimit)
    )
    .withSlot0(
      new Slot0Configs()
      .withKS(IntakeConstants.kRollerKS)
      .withKV(IntakeConstants.kRollerKV)
      .withKP(IntakeConstants.kRollerKP)
      .withKI(IntakeConstants.kRollerKI)
      .withKD(IntakeConstants.kRollerKD)
    );
    motorConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.rollerRatio;
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
   
    for (int i = 0; i < 5; ++i) {
      status = motorConfigurator.apply(motorConfigs);
      if (status.isOK()) break;
    }
    if (!status.isOK() ) {
      System.out.println("Could not apply configs, error code Status 1: " + status.toString());
    }
  }

  public void SetPivotConfig(){
    var motorConfigurator = PivotMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration(); 
    motorConfigs.withMotorOutput(
      new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Brake) 
    )
    .withCurrentLimits(
      new CurrentLimitsConfigs()
      .withStatorCurrentLimitEnable(IntakeConstants.kPivorMotorCurrentLimitEnable)
      .withStatorCurrentLimit(IntakeConstants.kPivotMotorCurrentLimit)
    )
    .withSlot0(
      new Slot0Configs()
      .withKS(IntakeConstants.kPivotKS)
      .withKV(IntakeConstants.kPivotKV)
      .withKA(IntakeConstants.kPivotKA)
      .withKP(IntakeConstants.kPivotKP)
      .withKI(IntakeConstants.kPivotKI)
      .withKD(IntakeConstants.kPivotKD)
    )
    .withMotionMagic(
      new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(IntakeConstants.kPivotMMCV)
      .withMotionMagicAcceleration(IntakeConstants.kPivotMMA)
      .withMotionMagicJerk(IntakeConstants.kPivotMMJ)
    );
    motorConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.pivotRatio;
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
   
    for (int i = 0; i < 5; ++i) {
      status = motorConfigurator.apply(motorConfigs);
      if (status.isOK()) break;
    }
    if (!status.isOK() ) {
      System.out.println("Could not apply configs, error code Status 1: " + status.toString());
    }
  }
// -----methods-------------------------------------------------------------
  public void setRollerSpeed(double speed){
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    RollerMotor.setControl(m_request.withVelocity(speed).withEnableFOC(true));
  }
  
  public void setPivotPoint(double position){
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0).withEnableFOC(true);

    PivotMotor.setControl(m_request.withPosition(position));
  }
    public double GetRollerVelocity(){
    return RollerMotor.getVelocity().getValueAsDouble();
  }
  public double GetPivotPosition(){
    return PivotMotor.getPosition().getValueAsDouble();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Roller Velocity", GetRollerVelocity());
    SmartDashboard.putNumber("Pivot Position", GetPivotPosition());
  }
}
