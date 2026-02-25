// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //-----------Subsystems------------------------------------------
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final HopperSubsystem m_hopper = new HopperSubsystem();
  private final KickerSubsystem m_kicker = new KickerSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  //-----------Drivetrain Setup-------------------------------------
   /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MaxSpeed * 0.1).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(DriveConstants.MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  //-----------Controller & Auto Chooser----------------------------
  //private final SendableChooser<Command> autoChooser;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  //----------------------------------------------------------------------------------------------
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Auto", autoChooser);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
    // Drivetrain will execute this command periodically
    drivetrain.applyRequest(() ->
      drive.withVelocityX(-m_driverController.getLeftY() * DriveConstants.MaxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(-m_driverController.getLeftX() * DriveConstants.MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(-m_driverController.getRightX() * DriveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
          )
        );
    /* 
    m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.SetVelocity(100), m_shooter));
    m_intake.setDefaultCommand(new RunCommand(()-> m_intake.setRollerSpeed(0),m_intake));
    */
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_driverController.a()
    .onTrue(
      new ParallelCommandGroup(
      (new RunCommand(() -> m_hopper.setDutyCycleOut(HopperConstants.m_HopperSpeed), m_hopper)),
      (new RunCommand(()-> m_kicker.setVelocity(KickerConstants.m_KickerVelocity), m_kicker))))
    .onFalse(
      new ParallelCommandGroup(
      (new RunCommand(() -> m_hopper.setNeutral(), m_hopper)),
      (new RunCommand(()-> m_kicker.setNeutral(), m_kicker)))
      );
    //m_driverController.b().onTrue(new RunCommand(()-> m_intake.setPivotPoint(IntakeConstants.m_PivotUp), m_intake));
    m_driverController.x().onTrue(new RunCommand(() -> m_shooter.SetVelocity(2500), m_shooter))
    .onFalse(new RunCommand(()-> m_shooter.setNeutral(), m_shooter));
    m_driverController.y();
    m_driverController.rightTrigger().onTrue(new RunCommand(()-> m_intake.setRollerSpeed(IntakeConstants.m_RollerVelocity),m_intake))
    .onFalse(new RunCommand(()-> m_intake.setRollerNeutral(),m_intake));
    

    //Reset Heading
    m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    //-----------SysID Stuffs------------------------------------------------------------------------------
    //m_driverController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    //m_driverController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    //----Drivetrain--------------------------------------------------------------------
    /* 
    m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    */
    //--Intake Pivot---------------------------------------------------------------------------
    /* 
    m_driverController.povUp().whileTrue(m_intake.PivotsysIDQuasistatic(Direction.kForward));
    m_driverController.povDown().whileTrue(m_intake.PivotsysIDQuasistatic(Direction.kReverse));
    m_driverController.povLeft().whileTrue(m_intake.PivotsysIdDynamic(Direction.kForward));
    m_driverController.povRight().whileTrue(m_intake.PivotsysIdDynamic(Direction.kReverse));
    */
    //--Intake Roller---------------------------------------------------------------------------
    /* 
    m_driverController.povUp().whileTrue(m_intake.RollersysIDQuasistatic(Direction.kForward));
    m_driverController.povDown().whileTrue(m_intake.RollersysIDQuasistatic(Direction.kReverse));
    m_driverController.povLeft().whileTrue(m_intake.RollersysIdDynamic(Direction.kForward));
    m_driverController.povRight().whileTrue(m_intake.RollersysIdDynamic(Direction.kReverse));
    */
    //--Kicker---------------------------------------------------------------------------
    /* */
    m_driverController.povUp().whileTrue(m_kicker.sysIDQuasistatic(Direction.kForward));
    m_driverController.povDown().whileTrue(m_kicker.sysIDQuasistatic(Direction.kReverse));
    m_driverController.povLeft().whileTrue(m_kicker.sysIdDynamic(Direction.kForward));
    m_driverController.povRight().whileTrue(m_kicker.sysIdDynamic(Direction.kReverse));
    
    //--Shooter---------------------------------------------------------------------------
    /* 
    m_driverController.povUp().whileTrue(m_shooter.sysIDQuasistatic(Direction.kForward));
    m_driverController.povDown().whileTrue(m_shooter.sysIDQuasistatic(Direction.kReverse));
    m_driverController.povLeft().whileTrue(m_shooter.sysIdDynamic(Direction.kForward));
    m_driverController.povRight().whileTrue(m_shooter.sysIdDynamic(Direction.kReverse));
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new RunCommand(()-> m_intake.setRollerSpeed(0), m_intake);//autoChooser.getSelected();
  }
}
