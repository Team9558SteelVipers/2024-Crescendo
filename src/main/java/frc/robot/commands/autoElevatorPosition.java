// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClawElevatorSubsystem;
import frc.robot.subsystems.ClawRotationSubsystem;

public class autoElevatorPosition extends Command {
 
  private ClawElevatorSubsystem m_elevatorSubsystem;
  private ClawRotationSubsystem m_ClawRotationSubsystem;
  private boolean targetPositionIsBottom;
  private Timer m_timer = new Timer();

  public autoElevatorPosition(ClawElevatorSubsystem elevatorSubsystem, ClawRotationSubsystem clawRotationSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_ClawRotationSubsystem = clawRotationSubsystem;
    addRequirements(elevatorSubsystem,clawRotationSubsystem);
    targetPositionIsBottom = true;
  }

  //Iterates through the positions
  @Override
  public void initialize() {
    targetPositionIsBottom = !targetPositionIsBottom;
     m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
     
    if (targetPositionIsBottom) {
      m_ClawRotationSubsystem.setClawRotationPiston(false);
      m_elevatorSubsystem.setElevatortoBotom();
    } else {
      m_elevatorSubsystem.setElevatorToTop();
      if (m_timer.get()> 0.125) {
        m_ClawRotationSubsystem.setClawRotationPiston(true);
      }
    } 
    //SmartDashboard.putBoolean("Piston extended", )
  }

  
  @Override
  public void end(boolean interrupted) {
    // m_elevatorSubsystem.setElevatorMotor(0);
    // m_ClawRotationSubsystem.setClawRotationPiston(false);
  }
  

  @Override
  public boolean isFinished() {

    // if (targetPositionIsBottom) return m_elevatorSubsystem.getElevatorPosition() < 1;
    return m_timer.get() > 0.6;

  }
}
