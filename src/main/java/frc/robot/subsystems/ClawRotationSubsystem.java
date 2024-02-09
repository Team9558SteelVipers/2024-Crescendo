// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawRotationSubsystem extends SubsystemBase {

  static DoubleSolenoid clawRotationPiston;
  
  
  public ClawRotationSubsystem() {
    clawRotationPiston = new DoubleSolenoid( , , );
  }// have to fill in modle type, foward port, reverse port

  public void setClawRotationPiston(Boolean clawRotationPistonValue) {
    if (clawRotationPistonValue == false){
      clawRotationPistonValue.set(DoubleSolenoid.Value.kReverse);
    } else if (clawRotationPistonValue == true){
      clawRotationPistonValue.set(DoubleSolenoid.Value.kForward);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
