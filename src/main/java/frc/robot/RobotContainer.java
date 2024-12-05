// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  // Replace with CommandPS4Controller or CommandJoystick if needed

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
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_autoChooser.addOption("Test Path", m_SwerveDriveTrain.getAutoPath("Test Auto"));
    SmartDashboard.putData(m_autoChooser);

    // Configure the trigger bindings
    configureBindings();
    configureDriveTrain();
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }

  private void configureDriveTrain() {
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
      ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
