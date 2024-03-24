// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequentialCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoElevatorPosition;
import frc.robot.commands.autoScoringCommand;
import frc.robot.commands.driveToAmpCommand;
import frc.robot.subsystems.ClawElevatorSubsystem;
import frc.robot.subsystems.ClawRotationSubsystem;
import frc.robot.subsystems.ClawScoringSubsystem;
import frc.robot.subsystems.CTRESwerve.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ampAuton extends SequentialCommandGroup {
  autoElevatorPosition m_AutoElevatorPosition;
  autoScoringCommand m_AutoScoringCommand;
  driveToAmpCommand m_DriveToAmpCommand;
  /** Creates a new ampAuton. */
  public ampAuton( ClawElevatorSubsystem clawElevatorSubsystem, ClawScoringSubsystem clawScoringSubsystem, CommandSwerveDrivetrain swerveDrivetrain, ClawRotationSubsystem clawRotationSubsystem) {
    autoElevatorPosition m_AutoElevatorPosition = new autoElevatorPosition(clawElevatorSubsystem, clawRotationSubsystem);
    autoScoringCommand m_AutoScoringCommand = new autoScoringCommand(clawScoringSubsystem);
    driveToAmpCommand m_DriveToAmpCommand = new driveToAmpCommand(swerveDrivetrain);
    addCommands(m_DriveToAmpCommand,m_AutoElevatorPosition,m_AutoScoringCommand,m_AutoElevatorPosition);
  }
}
