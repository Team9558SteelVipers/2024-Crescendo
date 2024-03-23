// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ScoringConstants.motorScoringSpeed;
import static frc.robot.Constants.ScoringConstants.scoringMotorPort;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawScoringSubsystem;


public class ScoringCommand extends Command {
  
 
  private static ClawScoringSubsystem m_clawScoringSubsystem;
  
  

  public ScoringCommand(ClawScoringSubsystem clawScoringSubsystem){
    
    m_clawScoringSubsystem = clawScoringSubsystem;
    addRequirements(clawScoringSubsystem);
  }

  
  @Override
  public void initialize() {

    
  }

 
  @Override
  public void execute() {

    
    m_clawScoringSubsystem.setIntakeMotorSpeed(motorScoringSpeed);

  }

  
  @Override
  public void end(boolean interrupted) {
    m_clawScoringSubsystem.setIntakeMotorSpeed(0.0);
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
