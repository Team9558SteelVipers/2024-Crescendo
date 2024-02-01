// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ScoringConstants.*;




public class ClawScoringSubsystem extends SubsystemBase {
  /** Creates a new ClawScoringSubsystems. */
  public TalonFX scoringMotor;
  public DigitalInput beamBreak;

  
  public ClawScoringSubsystem() {
    scoringMotor = new TalonFX(scoringMotorPort);
    beamBreak = new DigitalInput(beamBreakMotorPort);
  }

  public void setIntakeMotorSpeed(double speed) {
    scoringMotor.set(speed);
  }
  public double getIntakeMotorSpeed() {
    return scoringMotor.get();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
