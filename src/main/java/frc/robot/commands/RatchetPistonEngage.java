// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class RatchetPistonEngage extends Command {
  ClimberSubsystem m_ClimberSubsystem;
  
  /** Creates a new ratchetPistonCommand. */

  public RatchetPistonEngage(ClimberSubsystem climberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ClimberSubsystem.setRatchetPiston(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
