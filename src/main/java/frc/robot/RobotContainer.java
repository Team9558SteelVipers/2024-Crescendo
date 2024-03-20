// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.function.Supplier;

import org.ietf.jgss.Oid;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.commands.ClawRotationCommand;
import frc.robot.commands.ClawScoringCommand;
//import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ElevatorPosition;
//import frc.robot.commands.IntakeCommand;
import frc.robot.commands.NearestTrapCommand;
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

  //public ClawRotationCommand m_ClawRotationCommand = new ClawRotationCommand(m_ClawRotationSubsystem, operatorInput);
  public ClawScoringCommand m_ClawScoringCommand = new ClawScoringCommand(m_ClawScoringSubsystem);
  //public ClimberCommand m_ClimberCommand = new ClimberCommand(m_ClimberSubsystem, operatorInput);
  public ElevatorPosition m_ElevatorPosition = new ElevatorPosition(m_ClawElevatorSubsystem);
  //public IntakeCommand m_IntakeCommand = new IntakeCommand(m_IntakeSubsystem, operatorInput, m_ClawScoringSubsystem);
  public NearestTrapCommand m_NearestTrapCommand = new NearestTrapCommand(m_SwerveDriveTrain);

   private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */ 

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    m_SwerveDriveTrain.setDefaultCommand( // Drivetrain will execute this command periodically
        m_SwerveDriveTrain.applyRequest(() -> drive.withVelocityX(-operatorInput.getDriverController().getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-operatorInput.getDriverController().getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-operatorInput.getDriverController().getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    operatorInput.getDriverController().a().whileTrue(m_SwerveDriveTrain.applyRequest(() -> brake));
    operatorInput.getDriverController().b().whileTrue(m_SwerveDriveTrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-operatorInput.getDriverController().getLeftY(), -operatorInput.getDriverController().getLeftX()))));

    // reset the field-centric heading on left bumper press
    operatorInput.getDriverController().leftBumper(null).onTrue(m_SwerveDriveTrain.runOnce(() -> m_SwerveDriveTrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      m_SwerveDriveTrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    m_SwerveDriveTrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
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
