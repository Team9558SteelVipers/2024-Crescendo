// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.TrapPositions;

import frc.robot.subsystems.CTRESwerve.CommandSwerveDrivetrain;

public class NearestTrapCommand extends Command {
  private CommandSwerveDrivetrain m_SwerveDriveTrain;
  public double smallestOffset;
  /** Creates a new NearestTrapCommand. */
  public NearestTrapCommand(CommandSwerveDrivetrain swerveDriveTrain) {
    m_SwerveDriveTrain = swerveDriveTrain;
    addRequirements(swerveDriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
            for (int i = 0; i<6; i++) {
              
              smallestOffset = Math.min(
                                  Math.sqrt(Math.pow(Constants.TrapPositions.TrapX[i]-m_SwerveDriveTrain.getState().Pose.getX(),2)+
                                            Math.pow(Constants.TrapPositions.TrapY[i]-m_SwerveDriveTrain.getState().Pose.getY(),2)), 
                                  smallestOffset);
            }
          }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
