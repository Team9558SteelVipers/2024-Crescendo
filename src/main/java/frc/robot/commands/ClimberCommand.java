//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
//the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {

  private final ClimberSubsystem m_subsystem;

  public ClimberCommand(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if (OI.operatorController.getLeftY() > 0.1){
      m_subsystem.climberSpeed(OI.operatorController.getLeftY());
    } else {
      m_subsystem.climberSpeed(0);
    }
    
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
