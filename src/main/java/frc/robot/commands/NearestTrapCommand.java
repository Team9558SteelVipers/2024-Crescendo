// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CTRESwerve.CommandSwerveDrivetrain;

public class NearestTrapCommand extends Command {
  private CommandSwerveDrivetrain m_SwerveDriveTrain;
  public double smallestOffset;
  private PathPlannerPath pathToFollow;
  /** Creates a new NearestTrapCommand. */
  public NearestTrapCommand(CommandSwerveDrivetrain swerveDriveTrain) {
    m_SwerveDriveTrain = swerveDriveTrain;
    addRequirements(swerveDriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int smallestIndex = 0;
    for (int i = 0; i<6; i++) {
      if (smallestOffset > Math.sqrt(Math.pow(Constants.TrapPositions.TrapX[i]-m_SwerveDriveTrain.getState().Pose.getX(),2)+
                                     Math.pow(Constants.TrapPositions.TrapY[i]-m_SwerveDriveTrain.getState().Pose.getY(),2)))
      {
        smallestIndex = i;
        smallestOffset = Math.sqrt(Math.pow(Constants.TrapPositions.TrapX[i]-m_SwerveDriveTrain.getState().Pose.getX(),2)+
                                   Math.pow(Constants.TrapPositions.TrapY[i]-m_SwerveDriveTrain.getState().Pose.getY(),2));                 
      }
    }
      if (smallestIndex<=2 && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        switch (smallestIndex) {
          case 0:
              pathToFollow = PathPlannerPath.fromPathFile("LeftRedTrap");
            break;
          case 1:
              pathToFollow = PathPlannerPath.fromPathFile("LeftRedTrap");
            break;
          case 2:
              pathToFollow = PathPlannerPath.fromPathFile("LeftRedTrap");
            break;
          default:
            break;
        }
        
      } else if (smallestIndex >= 3 && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
        switch (smallestIndex) {
          case 3:
              pathToFollow = PathPlannerPath.fromPathFile("LeftRedTrap");
            break;
          case 4:
              pathToFollow = PathPlannerPath.fromPathFile("LeftRedTrap");
            break;
          case 5:
              pathToFollow = PathPlannerPath.fromPathFile("LeftRedTrap");
            break;
          default:
            break;
        }
        
      }
      AutoBuilder.followPath(pathToFollow).schedule();
      

    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return true;
  }
}
