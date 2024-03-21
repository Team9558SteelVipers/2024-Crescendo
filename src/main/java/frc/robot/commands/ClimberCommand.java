//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
//the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
boolean toplimitSwitchBoolean;

private final ClimberSubsystem m_subsystem;


public ClimberCommand(ClimberSubsystem subsystem) {
  m_subsystem = subsystem;
  addRequirements(subsystem);
}

@Override
public void initialize() {}

@Override
public void execute() { 
// Button Inputs, CHANGE ALL THE BUTTONS LATER
/* When the driver holds down the stop button, motor is set to 0
* knock knock... whos there... chicken! the end!
* first it checks if the stop, then up, then down buttons are held down
* Limit switches = stops if boundaries are close to being overstepped, 
* returns as true or false
* ps. the chicken got ran over 
*/
 
}

@Override
public void end(boolean interrupted) {}

@Override
public boolean isFinished() {;
    return false;
  }
}
