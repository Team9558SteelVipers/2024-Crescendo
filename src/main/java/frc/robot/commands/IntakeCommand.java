package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.ClawScoringSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeCommand extends CommandBase {
  
  private static IntakeSubsystem m_intakeSubsystem;
  private static OI m_operatorController;
  private static ClawScoringSubsystem m_clawScoringSubsystem;
  private static boolean beamBreakTriggered;
  

  public IntakeCommand(IntakeSubsystem intakeSubsystem, OI operator, ClawScoringSubsystem clawScoringSubsystem){
    m_intakeSubsystem = intakeSubsystem;
    m_operatorController = operator;
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
    return (!m_operatorController.getDRightBumper() || (beamBreakTriggered && m_clawScoringSubsystem.getEncoder()>=intakeDistanceConstant));
  }
}
