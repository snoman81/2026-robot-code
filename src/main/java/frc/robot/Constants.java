// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.generated.TunerConstants;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final CANBus rioBus = new CANBus("rio");

  public static class DriveConstants {
    public static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    
  }

  public static class IntakeConstants {
    //Roller Motor Configs
    public static final int kRollerMotorPort = 0;
    public static final boolean kRollerMotorCurrentLimitEnable = true;
    public static final int kRollerMotorCurrentLimit = 80;
    //Roller Motor Constants
    public static final double m_RollerVelocity = 0;
    //Pivor Motor Configs
    public static int kIntakePivotMotorPort = 0;
    public static AngularVelocity kPivotMMCV;
    public static final boolean kPivorMotorCurrentLimitEnable = false;
    public static final Current kPivotMotorCurrentLimit = null;
    public static final double kPivotKS = 0;
    public static final double kPivotKV = 0;
    public static final double kPivotKA = 0;
    public static final double kPivotKP = 0;
    public static final double kPivotKI = 0;
    public static final double kPivotKD = 0;
    public static final double kPivotMMA = 0;
    public static final double kPivotMMJ = 0;
    //Roller Motor Constants
    public static final double m_PivotUp = 0;
    


  }
  public static class KickerConstants {
    public static final int kKickerMotorPort = 0;
    public static final boolean kKickerMotorCurrentLimitEnable = true;
    public static final int kKickerMotorCurrentLimit = 60;
    public static final double m_KickerVelocity = 50; //rotations per second
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
  
    
  }
  public static class HopperConstants {

    public static final int kHopperMotorPort = 0;
    public static final boolean kHopperMotorCurrentLimitEnable = true;
    public static final int kHopperMotorCurrentLimit = 60;
    public static double m_HopperSpeed = 0.3; //out
  
    
  }
  public static class ShooterConstants {

    public static final int kMainMotorPort = 16;
    public static final int kFollowMotorPort = 0;
    public static final boolean kMotorCurrentLimitEnable = true;
    public static final int kMotorCurrentLimit = 120;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static double kMMA;
    public static double kMMJ;
    
  
    
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
