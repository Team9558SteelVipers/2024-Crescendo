// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ScoringConstants.*;


public class ClawScoringSubsystem extends SubsystemBase {
  /** Creates a new ClawScoringSubsystems. */
  public TalonFX scoringMotor;
  public DigitalInput beamBreak;

  public double encoderOffset = 0;
  CurrentLimitsConfigs clawConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(30);

  public ClawScoringSubsystem() {
    scoringMotor = new TalonFX(scoringMotorPort, "Canivore");
    beamBreak = new DigitalInput(beamBreakMotorPort);
    scoringMotor.getConfigurator().apply(clawConfigs);
  }

  public void setIntakeMotorSpeed(double speed) {
    scoringMotor.set(speed);
  }
  public double getIntakeMotorSpeed() {
    return scoringMotor.get();
  }

  public boolean getBeamBreak() {
    return !beamBreak.get();
  }

  public void resetEncoder() {
    encoderOffset = getEncoder();
  }

  public void setEncoder(double encoderValue) {
    scoringMotor.setPosition(encoderValue);
  }

  public double getEncoder(){
    return scoringMotor.getPosition().getValue() - encoderOffset;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
