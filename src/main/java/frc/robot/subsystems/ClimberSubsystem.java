// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
private static final int climberMotorPort = 0;
private final TalonSRX climberMotor; 

public ClimberSubsystem() {
  climberMotor = new TalonSRX(climberMotorPort);
}

public void climberUp (double speed){
  climberMotor.set(ControlMode.PercentOutput, speed);
}

public void climberDown(double speed){
  climberMotor.set(ControlMode.PercentOutput, -speed);
}

public void climberStop (){
  climberMotor.set(ControlMode.PercentOutput, 0);
}


  @Override
  public void periodic() {
  // This method will be called once per scheduler run
  }
}
