// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  // ------------Define Motors------------------
  public final TalonFX RollerMotor = new TalonFX(IntakeConstants.kRollerMotorPort, Constants.rioBus);
  public final TalonFX PivotMotor = new TalonFX(IntakeConstants.kIntakePivotMotorPort, Constants.rioBus);
  public final InterpolatingDoubleTreeMap speedtoRPM = new InterpolatingDoubleTreeMap();
  /** Creates a new Intake. */
  public IntakeSubsystem() {
    SetRollerConfig();
    SetPivotConfig();
    RPMMapFill();
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
      .withKA(IntakeConstants.kRollerKA)
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
      .withKG(IntakeConstants.kPivotKG)
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

    public void RPMMapFill(){ // Fill with intake testing 
    speedtoRPM.put(0.0,1000.0);
    speedtoRPM.put(1.5,1250.0);
    speedtoRPM.put(2.0,1500.0);
    speedtoRPM.put(2.5,1750.0);
    speedtoRPM.put(DriveConstants.MaxSpeed,2000.0);
    }
// -----methods-------------------------------------------------------------
  public void setRollerSpeed(double rps){
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    RollerMotor.setControl(m_request.withVelocity(rps).withEnableFOC(true));
  }

  public void setRollerNeutral (){
    RollerMotor.setControl(new NeutralOut()); 
  }
  
  public void setPivotPoint(double position){
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0).withEnableFOC(true);
    PivotMotor.setControl(m_request.withPosition(position));
  }
  public void setPivotOut(double output){
    PivotMotor.setControl(new DutyCycleOut(output));
  }
  public void setPivotNeutral(){
    PivotMotor.setControl(new NeutralOut());
  }
    public double getRollerVelocity(){
    return RollerMotor.getVelocity().getValueAsDouble();
  }
  public double getPivotPosition(){
    return PivotMotor.getPosition().getValueAsDouble();
  }
  public boolean getPivotatPoint(){
    return PivotMotor.getMotionMagicAtTarget().getValue();
  }
  public double getRollerVelErr(){
    return RollerMotor.getClosedLoopError().getValueAsDouble();
  }
//----SysID Methods---------------------------------------------------------
private final SysIdRoutine m_RollerSysIdRoutine = 
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("SysIDRoller_state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> RollerMotor.setControl(new VoltageOut(0).withOutput(volts.in(Volts))),
         null,
         this
      )
   );
private final SysIdRoutine m_PivotSysIdRoutine = 
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("SysIDPivot_state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> PivotMotor.setControl(new VoltageOut(0).withOutput(volts.in(Volts))),
         null,
         this
      )
   );

   public Command RollersysIDQuasistatic(SysIdRoutine.Direction direction){
    return m_RollerSysIdRoutine.quasistatic(direction);
   }
   public Command RollersysIdDynamic(SysIdRoutine.Direction direction) {
   return m_RollerSysIdRoutine.dynamic(direction);
  }
     public Command PivotsysIDQuasistatic(SysIdRoutine.Direction direction){
    return m_PivotSysIdRoutine.quasistatic(direction);
   }
   public Command PivotsysIdDynamic(SysIdRoutine.Direction direction) {
   return m_PivotSysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Roller Velocity", getRollerVelocity());
    SmartDashboard.putNumber("Roller Velocity Error", getRollerVelErr());
    SmartDashboard.putNumber("Pivot Position", getPivotPosition());
  }
}
