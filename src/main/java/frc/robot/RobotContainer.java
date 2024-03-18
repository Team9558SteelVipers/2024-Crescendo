// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.ietf.jgss.Oid;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClawRotationCommand;
import frc.robot.commands.ClawScoringCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ElevatorPosition;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.NearestTrapCommand;
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
  public OI operatorInput = new OI();

  public CommandSwerveDrivetrain m_SwerveDriveTrain = TunerConstants.DriveTrain;
  public IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  public ClawScoringSubsystem m_ClawScoringSubsystem = new ClawScoringSubsystem();
  public ClawRotationSubsystem m_ClawRotationSubsystem = new ClawRotationSubsystem();
  public ClawElevatorSubsystem m_ClawElevatorSubsystem = new ClawElevatorSubsystem();
  public VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

  public ClawRotationCommand m_ClawRotationCommand = new ClawRotationCommand(m_ClawRotationSubsystem, operatorInput);
  public ClawScoringCommand m_ClawScoringCommand = new ClawScoringCommand(m_ClawScoringSubsystem);
  public ClimberCommand m_ClimberCommand = new ClimberCommand(m_ClimberSubsystem, operatorInput);
  public ElevatorPosition m_ElevatorPosition = new ElevatorPosition(m_ClawElevatorSubsystem);
  public IntakeCommand m_IntakeCommand = new IntakeCommand(m_IntakeSubsystem, operatorInput, m_ClawScoringSubsystem);
  public NearestTrapCommand m_NearestTrapCommand = new NearestTrapCommand(m_SwerveDriveTrain);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  

  /** The  container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    


    
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.jsonCommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("SH-A-F1c-A ver2");
  }
}
