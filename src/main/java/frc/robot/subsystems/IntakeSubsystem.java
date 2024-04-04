// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
  
  TalonFX intakeMotor;

  CurrentLimitsConfigs intakeConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(30);
  
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(intakeMotorPort, "Canivore");
        intakeMotor.getConfigurator().apply(intakeConfigs);

  }

  public void setIntakeMotorSpeed(double speed) {
    intakeMotor.set(speed);
  }
  
  public double getIntakeMotorSpeed() {
    return intakeMotor.get();
  }

  @Override
  public void periodic() {
    
  }
}
