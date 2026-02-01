// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;



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

  public static class IntakeConstants {
    //Roller Motor Constants
    public static final int kRollerMotorPort = 0;
    public static final boolean kRollerMotorCurrentLimitEnable = true;
    public static final int kRollerMotorCurrentLimit = 80;
    
    //Pivot Motor Constants
    public static int kIntakePivotMotorPort = 0;
  }
  public static class KickerConstants {
    public static final int kKickerMotorPort = 0;
    public static final boolean kKickerMotorCurrentLimitEnable = true;
    public static final int kKickerMotorCurrentLimit = 60;
  
    
  }
  public static class HopperConstants {

    public static final int kHopperMotorPort = 0;
    public static final boolean kHopperMotorCurrentLimitEnable = true;
    public static final int kHopperMotorCurrentLimit = 60;
  
    
  }
  public static class ShooterConstants {

    public static final int kMainMotorPort = 0;
    public static final int kFollowMotorPort = 0;
    public static final boolean kMotorCurrentLimitEnable = true;
    public static final int kMotorCurrentLimit = 120;
    public static final double kS = 0;
    public static final double kV = 0.12;
    public static final double kA = 2.1;
    public static final double kP = 0.4;
    public static final double kI = 5;
    public static final double kD = 0;
    
  
    
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
