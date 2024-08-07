// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ClawRotationCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ElevatorPosition;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.InverseIntake;
import frc.robot.commands.InverseScoringCommand;
import frc.robot.commands.NearestTrapCommand;
import frc.robot.commands.RatchetPistonDisengage;
import frc.robot.commands.RatchetPistonEngage;
import frc.robot.commands.ScoringCommand;
import frc.robot.commands.autoElevatorPosition;
import frc.robot.commands.autoIntakeCommand;
import frc.robot.commands.autoInverseIntakeCommand;
import frc.robot.commands.autoScoringCommand;
import frc.robot.subsystems.ClawElevatorSubsystem;
import frc.robot.subsystems.ClawRotationSubsystem;
import frc.robot.subsystems.ClawScoringSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CTRESwerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.CTRESwerve.generated.TunerConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static OI operatorInput = new OI();

  public static final CommandSwerveDrivetrain m_SwerveDriveTrain = TunerConstants.DriveTrain;
  public static final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public static final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  public static final ClawScoringSubsystem m_ClawScoringSubsystem = new ClawScoringSubsystem();
  public static final ClawRotationSubsystem m_ClawRotationSubsystem = new ClawRotationSubsystem();
  public static final ClawElevatorSubsystem m_ClawElevatorSubsystem = new ClawElevatorSubsystem();
  public static final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

  public static final ClawRotationCommand m_ClawRotationCommand = new ClawRotationCommand(m_ClawRotationSubsystem);
  public static final ScoringCommand m_ScoringCommand = new ScoringCommand(m_ClawScoringSubsystem);
  public static final ClimberCommand m_ClimberCommand = new ClimberCommand(m_ClimberSubsystem);
  public static final ElevatorPosition m_ElevatorPosition = new ElevatorPosition(m_ClawElevatorSubsystem, m_ClawRotationSubsystem);
  public static final IntakeCommand m_IntakeCommand = new IntakeCommand(m_IntakeSubsystem, m_ClawScoringSubsystem);
  public static final InverseIntake m_InverseIntakeCommand = new InverseIntake(m_IntakeSubsystem, m_ClawScoringSubsystem);
  public static final InverseScoringCommand m_InverseScoringCommand = new InverseScoringCommand(m_ClawScoringSubsystem);
  public static final NearestTrapCommand m_NearestTrapCommand = new NearestTrapCommand(m_SwerveDriveTrain);
  public static final RatchetPistonEngage m_ratchetEngage = new RatchetPistonEngage(m_ClimberSubsystem);
  public static final RatchetPistonDisengage m_ratchetDisengage = new RatchetPistonDisengage(m_ClimberSubsystem);

  /* ====================================================================================== SWERVE DRIVE CONFIGURATION | START */
  // PARAMETERS
  private static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private static double PercentMinSpeed = 0.2;
  private static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private static double PercentLimit = 0.60; // base speed is percent of maxspeed
  private static double ZeroToMaxTime = 0.7; // time to reach max speed in seconds
  private static double PercentDeadband = 0.1;

  private static PhoenixPIDController HeadingController = new PhoenixPIDController(5, 0, 0);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  
  /* ======================================================================================== SWERVE DRIVE CONFIGURATION | END */
  
  private static double PercentGas = (1.0 - PercentLimit) > 0.0 ? 1.0 - PercentLimit : 0.0; // Make sure gas mulitplier doesn't become negative
  private static double PercentBrake = (PercentLimit - PercentMinSpeed) > 0.0 ? PercentLimit - PercentMinSpeed : 0.0; // Make sure PercentLimit >= PercentMinSpeed;
  private static double Acceleration = MaxSpeed/ZeroToMaxTime;
  
  private static SlewRateLimiter xVelRateLimited = new SlewRateLimiter(Acceleration);
  private static SlewRateLimiter yVelRateLimited = new SlewRateLimiter(Acceleration);
  
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * PercentDeadband).withRotationalDeadband(MaxAngularRate * PercentDeadband) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * PercentDeadband) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
  //private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    // m_SwerveDriveTrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     m_SwerveDriveTrain.applyRequest(() -> drive.withVelocityX(-operatorInput.getDriverController().getLeftY() * MaxSpeed) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(-operatorInput.getDriverController().getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(-operatorInput.getDriverController().getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ));

    // operatorInput.getDriverController().a().whileTrue(m_SwerveDriveTrain.applyRequest(() -> brake));
    // operatorInput.getDriverController().b().whileTrue(m_SwerveDriveTrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-operatorInput.getDriverController().getLeftY(), -operatorInput.getDriverController().getLeftX()))));

    //reset the field-centric heading on left bumper press
    // operatorInput.getDriverController().leftBumper().onTrue(m_SwerveDriveTrain.runOnce(() -> m_SwerveDriveTrain.seedFieldRelative()));
    
    operatorInput.getOperatorController().leftTrigger(0.5).whileTrue(m_IntakeCommand);
    operatorInput.getOperatorController().rightTrigger(0.5).whileTrue(m_InverseIntakeCommand);
    operatorInput.getOperatorController().x().onTrue(new InstantCommand(() ->
    {
      m_ElevatorPosition.toggleElevatorPosition();
    }));
    operatorInput.getOperatorController().rightBumper().whileTrue(m_ScoringCommand);
    operatorInput.getOperatorController().leftBumper().whileTrue(m_InverseScoringCommand);
    operatorInput.getOperatorController().a().onTrue(m_ratchetEngage);
    operatorInput.getOperatorController().b().onTrue(m_ratchetDisengage);
    operatorInput.getOperatorController().leftStick().whileTrue(m_ClimberCommand);
    
    
    //operatorInput.getOperatorController().leftBumper().runOnce(m_ClawRotationCommand);

    if (Utils.isSimulation()) {
      m_SwerveDriveTrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    //m_SwerveDriveTrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {

    m_ClawElevatorSubsystem.setDefaultCommand(m_ElevatorPosition);
    m_ClimberSubsystem.setDefaultCommand(m_ClimberCommand);
    NamedCommands.registerCommand("toggleElevator", new autoElevatorPosition(m_ClawElevatorSubsystem, m_ClawRotationSubsystem));
    NamedCommands.registerCommand("shoot", new autoScoringCommand(m_ClawScoringSubsystem));
    NamedCommands.registerCommand("intake", new autoIntakeCommand(m_IntakeSubsystem, m_ClawScoringSubsystem).withTimeout(4));
    NamedCommands.registerCommand("inverseIntake", new autoInverseIntakeCommand(m_IntakeSubsystem, m_ClawScoringSubsystem).withTimeout(2));

    m_autoChooser.setDefaultOption("Do nothing", new InstantCommand());
    m_autoChooser.addOption("2 note cycle", m_SwerveDriveTrain.getAutoPath("2 notes"));
    m_autoChooser.addOption("stop after amp", m_SwerveDriveTrain.getAutoPath("1 note"));
    m_autoChooser.addOption("pick up center note", m_SwerveDriveTrain.getAutoPath("1.5 notes"));
    m_autoChooser.addOption("Forward", m_SwerveDriveTrain.getAutoPath("Forward"));
    m_autoChooser.addOption("drop the glove", m_SwerveDriveTrain.getAutoPath("Drop the glove"));
    m_autoChooser.addOption("2.5 notes", m_SwerveDriveTrain.getAutoPath("2.5 notes"));
    SmartDashboard.putData(m_autoChooser);

    configureBindings();
    configureDrivetrain();
  }
   
  public static void rumbleControllers() {
    operatorInput.getDriverController().getHID().setRumble(RumbleType.kBothRumble, 1.0);
    operatorInput.getOperatorController().getHID().setRumble(RumbleType.kBothRumble, 1.0);
  }

  public static void stopRumbleControllers() {
    operatorInput.getDriverController().getHID().setRumble(RumbleType.kBothRumble, 0);
    operatorInput.getOperatorController().getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  private void configureDrivetrain() {
    driveFacing.HeadingController = HeadingController;
    
    operatorInput.getDriverController().x().onTrue(m_SwerveDriveTrain.runOnce(() -> 
    {
      m_SwerveDriveTrain.seedFieldRelative();
      m_SwerveDriveTrain.setHeadingToMaintain(m_SwerveDriveTrain.getCurrentRobotHeading());
    }
    ));
    
    operatorInput.getDriverController().y().onTrue(m_SwerveDriveTrain.runOnce(() -> m_SwerveDriveTrain.setHeadingToMaintain(m_SwerveDriveTrain.getOperatorForwardDirection())));
    //operatorInput.getDriverController().a().onTrue(m_SwerveDriveTrain.runOnce(() -> m_SwerveDriveTrain.setHeadingToMaintain(m_SwerveDriveTrain.getOperatorForwardDirection().rotateBy(new Rotation2d(-1.0, 0.0)))));
    operatorInput.getDriverController().b().onTrue(m_SwerveDriveTrain.runOnce(() -> 
    {
      double val = 1.0;
      if (DriverStation.getAlliance().get() == Alliance.Red)
      {
        val = -val;
      }
      m_SwerveDriveTrain.setHeadingToMaintain(m_SwerveDriveTrain.getOperatorForwardDirection().rotateBy(new Rotation2d(0.0, val)));
    }));
    
     // Boost
     operatorInput.getDriverController().rightBumper().onTrue(new InstantCommand(() ->
     {
       xVelRateLimited.reset(-operatorInput.getDriverController().getLeftY() * MaxSpeed * 1.33);
       yVelRateLimited.reset(-operatorInput.getDriverController().getLeftX() * MaxSpeed * 1.33);
     }));
 
     // Stop
     operatorInput.getDriverController().leftBumper().onTrue(new InstantCommand(() ->
     {
       xVelRateLimited.reset(0.0);
       yVelRateLimited.reset(0.0);
     }));

    m_SwerveDriveTrain.setDefaultCommand( // Drivetrain will execute this command periodically
      m_SwerveDriveTrain.applyRequest(

        // operatorInput.getDriverController(), // provide controller inputs to know when to use FieldCentricFacingAngle

        () -> drive
          .withVelocityX(
            xVelRateLimited.calculate( // control acceleration
              (-operatorInput.getDriverController().getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
              * (PercentLimit // limit base speed
              + (operatorInput.getDriverController().getRightTriggerAxis()*PercentGas) // Right Trigger to increase to max speed
              - (operatorInput.getDriverController().getLeftTriggerAxis()*PercentBrake)) // Left Trigger to decrease to min speed
            )
          ) 
          .withVelocityY(
            yVelRateLimited.calculate(
              (-operatorInput.getDriverController().getLeftX() * MaxSpeed) // Drive left with negative X (left)
              * (PercentLimit // limit base speed
              + (operatorInput.getDriverController().getRightTriggerAxis()*PercentGas) // Right Trigger to increase to max speed
              - (operatorInput.getDriverController().getLeftTriggerAxis()*PercentBrake)) // Left Trigger to decrease to min speed
            )
          ) 
          .withRotationalRate(-operatorInput.getDriverController().getRightX()*MaxAngularRate) // Drive counterclockwise with negative X (left)
       
       
          // () -> driveFacing
          // .withVelocityX(
          //   xVelRateLimited.calculate( // control acceleration
          //     (-operatorInput.getDriverController().getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
          //     * (PercentLimit // limit base speed
          //     + (operatorInput.getDriverController().getRightTriggerAxis()*PercentGas) // Right Trigger to increase to max speed
          //     - (operatorInput.getDriverController().getLeftTriggerAxis()*PercentBrake)) // Left Trigger to decrease to min speed
          //   )
          // )
          // .withVelocityY(
          //   yVelRateLimited.calculate(
          //     (-operatorInput.getDriverController().getLeftX() * MaxSpeed) // Drive left with negative X (left)
          //     * (PercentLimit // limit base speed
          //     + (operatorInput.getDriverController().getRightTriggerAxis()*PercentGas) // Right Trigger to increase to max speed
          //     - (operatorInput.getDriverController().getLeftTriggerAxis()*PercentBrake)) // Left Trigger to decrease to min speed
          //   )
          // ) 
          // .withTargetDirection(m_SwerveDriveTrain.getHeadingToMaintain()) // Maintain last known heading
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
   } 

  /** The  container for the robot. Contains subsystems, OI devices, and commands. */

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.jsonCommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  
}
