package frc.robot.commands;


import frc.robot.OI;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberCommand extends CommandBase {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
DigitalInput toplimitSwitch = new DigitalInput(0);
boolean toplimitSwitchBoolean;

private final ClimberSubsystem m_subsystem;
private final OI m_controller;

public ClimberCommand(ClimberSubsystem subsystem, OI driverController) {
  m_subsystem = subsystem;
  m_controller = driverController;
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
  toplimitSwitchBoolean = toplimitSwitch.get();
  if (m_controller.getDBButton()){
    m_subsystem.climberStop(0);
  }
  else if (m_controller.getDXButton()) {
    if (toplimitSwitchBoolean) {
      m_subsystem.climberUp(0);
    } 
  } else if (m_controller.getDYButton()){
    m_subsystem.climberDown(0);
  } else {
    //Do Nothing
  }
}

@Override
public void end(boolean interrupted) {}

@Override
public boolean isFinished() {;
    return false;
  }
}