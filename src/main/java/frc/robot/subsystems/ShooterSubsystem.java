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
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
    )
    .withSlot0(
      new Slot0Configs()
      .withKS(ShooterConstants.kS)
      .withKV(ShooterConstants.kV)
      .withKA(ShooterConstants.kA)
      .withKP(ShooterConstants.kP)
      .withKI(ShooterConstants.kI)
      .withKD(ShooterConstants.kD)
    );
    motorConfigs.Feedback.SensorToMechanismRatio = ShooterConstants.mechanismRatio;
    // Config main MOTOR FIRST
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motorConfigurator1.apply(motorConfigs);
      status = motorConfigurator2.apply(motorConfigs);
      if (status.isOK()) break;
    }
    if (!status.isOK() ) {
      System.out.println("Could not apply configs, error code Status 1: " + status.toString());
    }
    //Set Follow to Follow
    FollowMotor.setControl(new Follower(MainMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }
  //----SysID Methods---------------------------------------------------------
private final SysIdRoutine m_ShooterSysIdRoutine = 
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("SysIDShooter_State", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> MainMotor.setControl(new VoltageOut(0).withOutput(volts.in(Volts))),
         null,
         this
      )
   );

   public Command sysIDQuasistatic(SysIdRoutine.Direction direction){
    return m_ShooterSysIdRoutine.quasistatic(direction);
   }
   public Command sysIdDynamic(SysIdRoutine.Direction direction) {
   return m_ShooterSysIdRoutine.dynamic(direction);
  }
// -----methods-------------------------------------------------------------
  public void SetVelocity(double rpm){
    rpm = rpm/60;
    final VelocityVoltage m_request = 
    new VelocityVoltage(0).withSlot(0);

    MainMotor.setControl(m_request.withVelocity(rpm).withEnableFOC(true));
  }

  public void setNeutral (){
    MainMotor.setControl(new NeutralOut()); 
  }


  public double GetVelocity(){
    return MainMotor.getVelocity().getValueAsDouble() * 60;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", GetVelocity());
  
  }
}
