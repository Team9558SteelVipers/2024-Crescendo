// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ClimberConstants.*;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax climberMotor; 
  private final DoubleSolenoid ratchetPiston;
  
  public ClimberSubsystem() {
    climberMotor = new CANSparkMax(climberMotorPort, MotorType.kBrushless);
    ratchetPiston = new DoubleSolenoid(Constants.PCM_ID, PneumaticsModuleType.CTREPCM, Constants.ClimberConstants.ratchetPistonPort[1], Constants.ClimberConstants.ratchetPistonPort[0]);
    climberMotor.setInverted(false);
  }

  public void climberSpeed (double speed){
    climberMotor.set(speed*Constants.ClimberConstants.maxSpeed);
  }

  public void climberStop (){
    climberMotor.set(0);
  }

  public void setRatchetPiston(boolean pistonValue) {

    if (!pistonValue) {ratchetPiston.set(DoubleSolenoid.Value.kReverse);}
    else if (pistonValue) {ratchetPiston.set(DoubleSolenoid.Value.kForward);}
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
