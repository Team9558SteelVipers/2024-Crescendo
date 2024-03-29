// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawScoringSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import static frc.robot.Constants.IntakeConstants.*;

public class autoIntakeCommand extends Command {
  
  private static IntakeSubsystem m_intakeSubsystem;
  private static ClawScoringSubsystem m_clawScoringSubsystem;

  

  public autoIntakeCommand(IntakeSubsystem intakeSubsystem, ClawScoringSubsystem clawScoringSubsystem){
    m_intakeSubsystem = intakeSubsystem;
    m_clawScoringSubsystem = clawScoringSubsystem;
    addRequirements(intakeSubsystem, clawScoringSubsystem);
  }

  
  @Override
  public void initialize() {
      
    
  }

 
  @Override
  public void execute() {

    if (!m_clawScoringSubsystem.getBeamBreak()) {
      m_intakeSubsystem.setIntakeMotorSpeed(intakeMotorSpeed);
      m_clawScoringSubsystem.setIntakeMotorSpeed(clawIntakeSpeed);
    }
    else
    {
      m_intakeSubsystem.setIntakeMotorSpeed(0.0);
      m_clawScoringSubsystem.setIntakeMotorSpeed(0.0);
    }
  }

  
  @Override
  public void end(boolean interrupted) {
      m_intakeSubsystem.setIntakeMotorSpeed(0.0);
      m_clawScoringSubsystem.setIntakeMotorSpeed(0.0);
  }

  
  @Override
  public boolean isFinished() {
    return m_clawScoringSubsystem.getBeamBreak();
  }
}
