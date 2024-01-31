package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawElevatorSubsystem;

public class ElevatorPosition extends CommandBase {
 
  private static ClawElevatorSubsystem m_elevatorSubsystem;
  private static int elevatorPosition;

  public ElevatorPosition(ClawElevatorSubsystem elevatorSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    elevatorPosition = 0;
    addRequirements(elevatorSubsystem);
  }

  //Iterates through the positions
  @Override
  public void initialize() {
    elevatorPosition = m_elevatorSubsystem.getElevatorPosition();
    
    if (m_elevatorSubsystem.getElevatorPosition() == 0) {m_elevatorSubsystem.setElevatorMotorPosition(1);}
    else if (m_elevatorSubsystem.getElevatorPosition() == 1) {m_elevatorSubsystem.setElevatorMotorPosition(2);}
    else if (m_elevatorSubsystem.getElevatorPosition() == 2) {m_elevatorSubsystem.setElevatorMotorPosition(0);}

    m_elevatorSubsystem.setElevatorMotorPosition(elevatorPosition);
    
  }

  
  @Override
  public void execute() {}


  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
