// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawElevatorSubsystem;
import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorPosition extends Command {
 
  private ClawElevatorSubsystem m_elevatorSubsystem;
  private boolean atBottom;
  private PIDController pidController = new PIDController(1, 0, 0);


  //-0.307617 bottom
  // -21.181641 top

  public ElevatorPosition(ClawElevatorSubsystem elevatorSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    atBottom = true;
    addRequirements(elevatorSubsystem);
  }

  //Iterates through the positions
  @Override
  public void initialize() {
    if (atBottom){
     
      m_elevatorSubsystem.setElevatorMotor(pidController.calculate(elevatorHeights[1]-elevatorHeights[0]));
    } else if (!atBottom) {
      m_elevatorSubsystem.setElevatorMotor(pidController.calculate(elevatorHeights[0]-elevatorHeights[1]));
    }
  }
  

  @Override
  public void execute() {
    
  }

  
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.setElevatorMotor(0);
  }
  

  @Override
  public boolean isFinished() {
    return false;
  }
}
