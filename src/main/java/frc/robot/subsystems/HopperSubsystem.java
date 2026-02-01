// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {
  public final TalonFX HopperMotor = new TalonFX(HopperConstants.kHopperMotorPort, Constants.rioBus);
  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {

  }
    public void SetkickerConfig(){
    var motorConfigurator = HopperMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();  
    motorConfigs
    .withMotorOutput(
    new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Coast)
    )
    .withCurrentLimits(
    new CurrentLimitsConfigs()
    .withStatorCurrentLimitEnable(HopperConstants.kHopperMotorCurrentLimitEnable)
    .withStatorCurrentLimit(HopperConstants.kHopperMotorCurrentLimit)
    );
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
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
