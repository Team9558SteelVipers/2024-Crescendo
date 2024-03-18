// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawElevatorSubsystem;

public class ElevatorPosition extends Command {
 
  private ClawElevatorSubsystem m_elevatorSubsystem;
  private int elevatorPosition;
  private PIDController pidController = new PIDController(0, 0, 0);

  public ElevatorPosition(ClawElevatorSubsystem elevatorSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    elevatorPosition = 0;
    addRequirements(elevatorSubsystem);
  }

  //Iterates through the positions
  @Override
  public void initialize() {}
  

  @Override
  public void execute() {
    double speed = pidController.calculate(m_elevatorSubsystem.getEncoderPosition());
    m_elevatorSubsystem.setElevatorMotor(speed);
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
