package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.IntakeSubsystem;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeCommand extends CommandBase {
  
  private static IntakeSubsystem m_IntakeSubsystem;
  private static OI operatorController;
  

  public IntakeCommand(IntakeSubsystem intakeSubsystem, OI operator){
    m_IntakeSubsystem = intakeSubsystem;
    operatorController = operator;
    addRequirements(intakeSubsystem);
  }

  
  @Override
  public void initialize() {
    m_IntakeSubsystem.setIntakeMotorSpeed(intakeMotorSpeed);
  }

 
  @Override
  public void execute() {}

  
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setIntakeMotorSpeed(0.0);
  }

  
  @Override
  public boolean isFinished() {
    return !operatorController.getDRightBumper();
  }
}
