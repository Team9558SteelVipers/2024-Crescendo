// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawElevatorSubsystem;
import frc.robot.subsystems.ClawRotationSubsystem;

import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorPosition extends Command {
 
  private ClawElevatorSubsystem m_elevatorSubsystem;
  private ClawRotationSubsystem m_ClawRotationSubsystem;
  private boolean targetPositionIsBottom;
  private Timer m_timer = new Timer();
  


  //-0.307617 bottom
  // -21.181641 top

  public ElevatorPosition(ClawElevatorSubsystem elevatorSubsystem, ClawRotationSubsystem clawRotationSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_ClawRotationSubsystem = clawRotationSubsystem;
    addRequirements(elevatorSubsystem,clawRotationSubsystem);
  }

  //Iterates through the positions
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    if (m_elevatorSubsystem.isAtBottom())
    {
      targetPositionIsBottom = false;
    }
    else
    {
      targetPositionIsBottom = true;
    }
  }
  

  @Override
  public void execute() {
    if (m_timer.get() > 0.25){
      //m_ClawRotationSubsystem.setClawRotationPiston(true);
    }
    if (targetPositionIsBottom)
    {
      m_elevatorSubsystem.setElevatortoBotom();

    }
    else
    {
      m_elevatorSubsystem.setElevatorToTop();
    }

    SmartDashboard.putBoolean("Target is bottom: ", targetPositionIsBottom);
    SmartDashboard.putNumber("Encoder Position: ", m_elevatorSubsystem.getEncoderPosition());
    SmartDashboard.putNumber("Timer Value: ", m_timer.get());
  }

  
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.setElevatorMotor(0);
    m_ClawRotationSubsystem.setClawRotationPiston(false);
  }
  

  @Override
  public boolean isFinished() {

    // if (targetPositionIsBottom) return m_elevatorSubsystem.getElevatorPosition() < 1;
    return m_elevatorSubsystem.getEncoderPosition() > 19.5;

  }
}
