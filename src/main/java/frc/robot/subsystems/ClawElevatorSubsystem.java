// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ElevatorConstants.*;


public class ClawElevatorSubsystem extends SubsystemBase {

  TalonFX elevatorMotor;
  static DoubleSolenoid ratchetPiston;
  // static PIDController ElevatorPID;
  private static int currentPosition;
  private static boolean isAtBottom;
  // MotorOutputConfigs motorConfig;
  
  
  public ClawElevatorSubsystem() {
    elevatorMotor = new TalonFX(clawElevatorPort, "Canivore");
    ratchetPiston = new DoubleSolenoid(Constants.PCM_ID, PneumaticsModuleType.CTREPCM, ratchetPistonPort[0], ratchetPistonPort[1]);
    
    // Modify Config First
    // motorConfig = new MotorOutputConfigs();
    // motorConfig.PeakForwardDutyCycle = 0.3;
    // motorConfig.PeakReverseDutyCycle = -0.3;

    // Use this for On-Board PID Control
    Slot0Configs elevatorPID = new Slot0Configs();
    elevatorPID.kP = Constants.ElevatorConstants.kElevatorP;
    elevatorPID.kI = Constants.ElevatorConstants.kElevatorI;
    elevatorPID.kD = Constants.ElevatorConstants.kElevatorD;
    elevatorPID.kG = Constants.ElevatorConstants.kElevatorG; // motor output to beat gravity
    elevatorPID.GravityType = GravityTypeValue.Elevator_Static;
    
    TalonFXConfiguration elevatorConfigs = new TalonFXConfiguration();
    elevatorConfigs.CurrentLimits.StatorCurrentLimit = Constants.kStatorCurrentLimit;
    elevatorConfigs.Slot0 = elevatorPID;
    
    // Then Apply Config
    // elevatorMotor.getConfigurator().apply(motorConfig);
    elevatorMotor.getConfigurator().apply(elevatorConfigs);
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    elevatorMotor.setPosition(0.0);
    elevatorMotor.setInverted(true);

    // ElevatorPID = new PIDController(P, I, D);   
    isAtBottom = true;
  }

  // Uses the PID to calculate motor speed based on how far it is from desired position
  public void setElevatorMotorPosition(double position){
    elevatorMotor.setControl(new PositionVoltage(position));
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

  public void setElevatorToTop()
  {
    setElevatorMotorPosition(29); // TODO save as constant
    isAtBottom = false;
  }

  public boolean isAtBottom()
  {
    return isAtBottom;
  }
  
  public void setElevatortoBotom()
  {
    setElevatorMotorPosition(0); // TODO save as constant
    isAtBottom = true;
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