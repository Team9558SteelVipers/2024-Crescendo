// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.ClawRotationSubsystem;

public class ClawRotationCommand extends Command {

  private final ClawRotationSubsystem m_clawRotationSubsystem;
  private boolean clawRotationPistonValue;
  private final XboxController m_operatorController;
 
  public ClawRotationCommand(ClawRotationSubsystem clawRotationSubsystem, OI operatorInput) {
    m_clawRotationSubsystem = clawRotationSubsystem;
    m_operatorController = operatorInput.getDriverController();
    addRequirements(clawRotationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
