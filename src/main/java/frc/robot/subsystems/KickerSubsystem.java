// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KickerConstants;

public class KickerSubsystem extends SubsystemBase {
  // ------------Define Motors------------------
  public final TalonFX KickerMotor = new TalonFX(KickerConstants.kKickerMotorPort, Constants.rioBus);
  /** Creates a new KickerSubsystem. */
  public KickerSubsystem() {
    SetkickerConfig();
  }
  
// -----cfgs-------------------------------------------------------------
    public void SetkickerConfig(){
    var motorConfigurator = KickerMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();  
    motorConfigs
    .withMotorOutput(
      new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Brake)
    )
    .withCurrentLimits(
      new CurrentLimitsConfigs()
      .withStatorCurrentLimitEnable(KickerConstants.kKickerMotorCurrentLimitEnable)
      .withStatorCurrentLimit(KickerConstants.kKickerMotorCurrentLimit)
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

   public void setDutyCycleOut(double speed) {
    final DutyCycleOut m_request = new DutyCycleOut(0).withEnableFOC(true);

    KickerMotor.setControl(m_request.withOutput(speed));
   }

    public double GetOutput(){
    return KickerMotor.getDutyCycle().getValueAsDouble();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Kicker Output", GetOutput());
  }
}
