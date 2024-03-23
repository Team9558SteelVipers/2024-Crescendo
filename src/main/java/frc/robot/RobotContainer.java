// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;


import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ClawRotationCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ElevatorPosition;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.InverseIntake;
import frc.robot.commands.NearestTrapCommand;
import frc.robot.commands.ScoringCommand;
import frc.robot.subsystems.ClawElevatorSubsystem;
import frc.robot.subsystems.ClawRotationSubsystem;
import frc.robot.subsystems.ClawScoringSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CTRESwerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.CTRESwerve.Telemetry;
import frc.robot.subsystems.CTRESwerve.generated.TunerConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public OI operatorInput = new OI();

  public CommandSwerveDrivetrain m_SwerveDriveTrain = TunerConstants.DriveTrain;
  public IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  public ClawScoringSubsystem m_ClawScoringSubsystem = new ClawScoringSubsystem();
  public ClawRotationSubsystem m_ClawRotationSubsystem = new ClawRotationSubsystem();
  public ClawElevatorSubsystem m_ClawElevatorSubsystem = new ClawElevatorSubsystem();
  public VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

  public ClawRotationCommand m_ClawRotationCommand = new ClawRotationCommand(m_ClawRotationSubsystem);
  public ScoringCommand m_ScoringCommand = new ScoringCommand(m_ClawScoringSubsystem);
  public ClimberCommand m_ClimberCommand = new ClimberCommand(m_ClimberSubsystem);
  public ElevatorPosition m_ElevatorPosition = new ElevatorPosition(m_ClawElevatorSubsystem, m_ClawRotationSubsystem);
  public IntakeCommand m_IntakeCommand = new IntakeCommand(m_IntakeSubsystem, m_ClawScoringSubsystem);
  public InverseIntake m_InverseIntakeCommand = new InverseIntake(m_IntakeSubsystem, m_ClawScoringSubsystem);
  public NearestTrapCommand m_NearestTrapCommand = new NearestTrapCommand(m_SwerveDriveTrain);

  /* ====================================================================================== SWERVE DRIVE CONFIGURATION | START */
  // PARAMETERS
  private static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private static double PercentMinSpeed = 0.2;
  private static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private static double PercentLimit = 0.60; // base speed is percent of maxspeed
  private static double ZeroToMaxTime = 0.7; // time to reach max speed in seconds
  private static double PercentDeadband = 0.1;

  private static PhoenixPIDController HeadingController = new PhoenixPIDController(5, 0, 0);
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
  
  private final Telemetry logger = new Telemetry(MaxSpeed);

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
    //operatorInput.getOperatorController().leftBumper().runOnce(m_ClawRotationCommand);


    if (Utils.isSimulation()) {
      m_SwerveDriveTrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    m_SwerveDriveTrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    m_ClawElevatorSubsystem.setDefaultCommand(m_ElevatorPosition);

    configureBindings();
    configureDrivetrain();
  }

  private void configureDrivetrain() {
    driveFacing.HeadingController = HeadingController;
    
    operatorInput.getDriverController().x().onTrue(m_SwerveDriveTrain.runOnce(() -> 
    {
      m_SwerveDriveTrain.seedFieldRelative();
    }
    ));
    
    operatorInput.getDriverController().y().onTrue(m_SwerveDriveTrain.runOnce(() -> m_SwerveDriveTrain.setHeadingToMaintain(new Rotation2d(0.0, 1.0))));
    operatorInput.getDriverController().a().onTrue(m_SwerveDriveTrain.runOnce(() -> m_SwerveDriveTrain.setHeadingToMaintain(new Rotation2d(0.0, -1.0))));


    //m_drivetrainSubsystem.setHeadingToMaintain(m_drivetrainSubsystem.getCurrentRobotHeading());

    m_SwerveDriveTrain.setDefaultCommand( // Drivetrain will execute this command periodically
      m_SwerveDriveTrain.applyRequest(

        operatorInput.getDriverController(), // provide controller inputs to know when to use FieldCentricFacingAngle

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
          .withRotationalRate(-operatorInput.getDriverController().getRightX()*MaxAngularRate), // Drive counterclockwise with negative X (left)
       
       
          () -> driveFacing
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
          .withTargetDirection(m_SwerveDriveTrain.getHeadingToMaintain()) // Maintain last known heading
      )
    );
  }

  public Command getAutonomousCommand() {
    
    try {
      return m_SwerveDriveTrain.getAutoPath("Forward");
    } catch (Exception e){
      return null;
    }
    
  }
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  

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
 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
}
