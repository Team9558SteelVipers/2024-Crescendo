// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ClawScoringSubsystem;

public class ClawScoringCommand extends Command {
  
  private static ClawScoringSubsystem m_subsystem;
  
  public ClawScoringCommand(ClawScoringSubsystem clawScoringSubsystem) {
    m_subsystem = clawScoringSubsystem;
    addRequirements(clawScoringSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Checks if there is a beam break (COULD BE CHANGED)
    //Unsure if GetEncoder is updating..
    if (m_subsystem.getBeamBreak()) {
      if (m_subsystem.getRingPosition() >= IntakeConstants.intakeDistanceConstant) {
        m_subsystem.setIntakeMotorSpeed(0);
      } else {
        m_subsystem.setIntakeMotorSpeed(1);
      }
    }
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
