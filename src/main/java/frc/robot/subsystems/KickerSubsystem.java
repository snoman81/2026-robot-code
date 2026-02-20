// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
      .withNeutralMode(NeutralModeValue.Coast)
    )
    .withCurrentLimits(
      new CurrentLimitsConfigs()
      .withStatorCurrentLimitEnable(KickerConstants.kKickerMotorCurrentLimitEnable)
      .withStatorCurrentLimit(KickerConstants.kKickerMotorCurrentLimit)
    )
    .withSlot0(
      new Slot0Configs()
      .withKS(KickerConstants.kS)
      .withKV(KickerConstants.kV)
      .withKA(KickerConstants.kA)
      .withKP(KickerConstants.kP)
      .withKI(KickerConstants.kI)
      .withKD(KickerConstants.kD)
    );
    motorConfigs.Feedback.SensorToMechanismRatio = KickerConstants.mechanismRatio;
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
//----SysID Methods---------------------------------------------------------
private final SysIdRoutine m_KickerSysIdRoutine = 
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("SysIDKicker_state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> KickerMotor.setControl(new VoltageOut(0).withOutput(volts.in(Volts))),
         null,
         this
      )
   );

   public Command sysIDQuasistatic(SysIdRoutine.Direction direction){
    return m_KickerSysIdRoutine.quasistatic(direction);
   }
   public Command sysIdDynamic(SysIdRoutine.Direction direction) {
   return m_KickerSysIdRoutine.dynamic(direction);
  }
  //------Methods----------------------------------------------------------------------
   public void setVelocity(double rps) {
     rps = rps/60;
    final VelocityVoltage m_request = 
    new VelocityVoltage(0).withSlot(0);

    KickerMotor.setControl(m_request.withVelocity(rps).withEnableFOC(true));
   }

    public double getVelocity(){
    return KickerMotor.getVelocity().getValueAsDouble() * 60;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Kicker Velocity", getVelocity());
  }
}
