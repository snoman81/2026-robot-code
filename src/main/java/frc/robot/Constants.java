// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.CANBus;

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

  public static final class TagLists {
      public static final List<Integer> blueTags = Arrays.asList(18,21,24,25,26,27);
      public static final List<Integer> redTags = Arrays.asList(2,5,8,9,10,11);
  }

  public static class IntakeConstants {
    //Roller Motor Configs
    public static final int kRollerMotorPort = 31;
    public static final boolean kRollerMotorCurrentLimitEnable = true;
    public static final int kRollerMotorCurrentLimit = 80;    
    public static final double kRollerKS = 0.22757;
    public static final double kRollerKV = 0.15;//0.11787;
    public static final double kRollerKA = 0.00438;
    public static final double kRollerKP = 0.75;
    public static final double kRollerKI = 0.05;
    public static final double kRollerKD = 0;
    public static final double rollerRatio = 1.33;
    //Roller Motor Constants
    public static final double m_RollerVelocity = -25;
    //Pivor Motor Configs
    public static final int kIntakePivotMotorPort = 32;
    public static final boolean kPivorMotorCurrentLimitEnable = false;
    public static final double kPivotMotorCurrentLimit = 80;
    public static final double kPivotKS = 0.13865;
    public static final double kPivotKV = 0.10193;
    public static final double kPivotKA = 0.011877;
    public static final double kPivotKG = 0.067353;
    public static final double kPivotKP = 66.513;
    public static final double kPivotKI = 0;
    public static final double kPivotKD = 1.1819;
    public static final double kPivotMMCV = 2;
    public static final double kPivotMMA = 2;
    public static final double kPivotMMJ = 3;   
    public static final double pivotRatio = 50;
    //Pivot Motor Setpoints
    public static final double m_PivotUp = 0;
    public static final double m_PivotDown = 0.25;
    

  }
  public static class KickerConstants {
    public static final int kKickerMotorPort = 35;
    public static final boolean kKickerMotorCurrentLimitEnable = true;
    public static final int kKickerMotorCurrentLimit = 60;
    public static final double m_KickerVelocity = 15;
    public static final double kS = 0.1194;
    public static final double kV = 0.5;//0.11295;
    public static final double kA = 0.008;
    public static final double kP = 2.5;
    public static final double kI = 0.1;
    public static final double kD = 0;
    public static final double mechanismRatio = 5;
  
  }
  public static class HopperConstants {

    public static final int kHopperMotorPort = 30;
    public static final boolean kHopperMotorCurrentLimitEnable = true;
    public static final int kHopperMotorCurrentLimit = 60;
    public static double m_HopperSpeed = 0.50; //out
  
  }
  public static class ShooterConstants {

    public static final int kMainMotorPort = 33;
    public static final int kFollowMotorPort = 34;
    public static final boolean kMotorCurrentLimitEnable = true;
    public static final int kMotorCurrentLimit = 120;
    public static final double kS = 0.076416;
    public static final double kV = 0.13126;
    public static final double kA = 0.021383;
    public static final double kP = 0.17515;
    public static final double kI = 0.025;
    public static final double kD = 0;
    public static final double mechanismRatio = 1.28;// count to get right value
    
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
