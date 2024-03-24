// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CTRESwerve.CommandSwerveDrivetrain;

public class driveToAmpCommand extends Command {

  CommandSwerveDrivetrain m_SwerveDrivetrain;
  Timer m_timer = new Timer();
    
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  /** Creates a new driveToAmpCommand. */
  public driveToAmpCommand(CommandSwerveDrivetrain swerveDrivetrain) {
    m_SwerveDrivetrain = swerveDrivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_SwerveDrivetrain.applyRequest(
    () -> drive
      .withVelocityX(0.2) 
      .withVelocityY(-0.2)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > 0.4;
  }
}
