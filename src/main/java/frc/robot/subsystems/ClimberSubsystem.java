// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
private static final int climberMotorPort = 0;
private final TalonSRX climberMotor; 
private final DoubleSolenoid ratchetPiston;

public ClimberSubsystem() {
  climberMotor = new TalonSRX(climberMotorPort);
  ratchetPiston = new DoubleSolenoid(Constants.PCM_ID, PneumaticsModuleType.CTREPCM, Constants.ClimberConstants.ratchetPistonPort[0], Constants.ClimberConstants.ratchetPistonPort[1]);
  climberMotor.setInverted(true);
}

public void climberSpeed (double speed){
  climberMotor.set(ControlMode.PercentOutput, speed*Constants.ClimberConstants.maxSpeed);
}

public void climberStop (){
  climberMotor.set(ControlMode.PercentOutput, 0);
}


  @Override
  public void periodic() {
  // This method will be called once per scheduler run
  }
}
