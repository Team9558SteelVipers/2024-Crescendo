// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants.*;


public class ClawElevatorSubsystem extends SubsystemBase {

  static TalonFX elevatorMotor;
  static DoubleSolenoid ratchetPiston;
  static PIDController ElevatorPID;
  private static int currentPosition;

  
  public ClawElevatorSubsystem() {
    elevatorMotor = new TalonFX(clawElevatorPort);
    ratchetPiston = new DoubleSolenoid(ratchetPistonPort[0], PneumaticsModuleType.CTREPCM, ratchetPistonPort[1], ratchetPistonPort[2]);


    ElevatorPID = new PIDController(P, I, D);
  }

  // Uses the PID to calculate motor speed based on how far it is from desired position
  public void setElevatorMotorPosition(int index){
    setElevatorMotor(ElevatorPID.calculate(elevatorHeights[index] - getEncoderPosition()));
    currentPosition = index;
  }


  public double getEncoderPosition() {
    return elevatorMotor.getPosition().getValue();
  }


  public void setEncoderPosition() {
    elevatorMotor.setPosition(0);
  }


  public void setElevatorMotor(double speed) {
    elevatorMotor.set(speed);
  }



  public void setRatchetPiston(int pistonValue) {

    if (pistonValue == 0){ratchetPiston.set(DoubleSolenoid.Value.kOff);}
    if (pistonValue == -1){ratchetPiston.set(DoubleSolenoid.Value.kReverse);}
    if (pistonValue == 1){ratchetPiston.set(DoubleSolenoid.Value.kForward);}
    
  }

  public int getElevatorPosition() {
    return currentPosition;
  }




  @Override
  public void periodic() {
    
  }
} 