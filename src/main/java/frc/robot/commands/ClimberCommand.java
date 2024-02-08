// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.OI;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/** An example command that uses an example subsystem. */
public class ClimberCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final ClimberSubsystem m_subsystem;
  public boolean isAuto = true;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param i
   */
  public ClimberCommand(ClimberSubsystem subsystem, int time) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }
        public class ClimbUp {
          private final Climber climber;
          public ClimbUp(Climber subsystem) {
            this.climber = subsystem;
            addRequirements(climber);
        }
        public class ClimbDown extends CommandBase{
         private final Climber climber;
 
        }
          
          public ClimbDown(Climber subsystem) {
            this.climber = subsystem;
            addRequirements(climber);

        }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (OI.getController().x().getAsBoolean()) {
      m_subsystem.climberMotorSpeed(0.5);
    } else if (OI.getController().y().getAsBoolean()){
      m_subsystem.climberMotorSpeed(-0.5);
    } else {
      //Do Nothing
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {;
    if (finished = true) {
      isAuto = false;
    }
    return finished;
  }
}
