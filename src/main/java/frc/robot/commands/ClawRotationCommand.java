// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawRotationSubsystem;

public class ClawRotationCommand extends Command {

  private final ClawRotationSubsystem m_clawRotationSubsystem;
  private Timer m_timer = new Timer();
  
  public ClawRotationCommand(ClawRotationSubsystem clawRotationSubsystem) {
    m_clawRotationSubsystem = clawRotationSubsystem;
    addRequirements(clawRotationSubsystem);
    
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
    if (m_timer.get() > 0.25){
      m_clawRotationSubsystem.setClawRotationPiston(true);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_clawRotationSubsystem.setClawRotationPiston(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
