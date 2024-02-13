// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.ClawScoringSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeCommand extends Command {
  
  private static IntakeSubsystem m_intakeSubsystem;
  private static XboxController m_operatorController;
  private static ClawScoringSubsystem m_clawScoringSubsystem;
  private static boolean beamBreakTriggered;
  

  public IntakeCommand(IntakeSubsystem intakeSubsystem, OI operatorInput, ClawScoringSubsystem clawScoringSubsystem){
    m_intakeSubsystem = intakeSubsystem;
    m_operatorController = operatorInput.getDriverController();
    m_clawScoringSubsystem = clawScoringSubsystem;
    beamBreakTriggered = false;
    addRequirements(intakeSubsystem, clawScoringSubsystem);
  }

  
  @Override
  public void initialize() {
    m_intakeSubsystem.setIntakeMotorSpeed(intakeMotorSpeed);
    m_clawScoringSubsystem.setIntakeMotorSpeed(clawIntakeSpeed);
  }

 
  @Override
  public void execute() {
    if (!m_clawScoringSubsystem.getBeamBreak()) {
      m_clawScoringSubsystem.resetEncoder();
      beamBreakTriggered = true;
    }

  }

  
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setIntakeMotorSpeed(0.0);
    m_clawScoringSubsystem.setIntakeMotorSpeed(0.0);
  }

  
  @Override
  public boolean isFinished() {
    return (!m_operatorController.getRightBumper() || (beamBreakTriggered && m_clawScoringSubsystem.getEncoder()>=intakeDistanceConstant));
  }
}
