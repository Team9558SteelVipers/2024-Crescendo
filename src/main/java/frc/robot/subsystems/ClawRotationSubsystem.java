// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawRotationSubsystem extends SubsystemBase {

  static DoubleSolenoid clawRotationPiston;
  
  
  public ClawRotationSubsystem() {
    clawRotationPiston = new DoubleSolenoid(Constants.PCM_ID, PneumaticsModuleType.CTREPCM,Constants.ScoringConstants.ClawPistonPorts[0],Constants.ScoringConstants.ClawPistonPorts[1]);
    
  }

  public void setClawRotationPiston(Boolean clawRotationPistonValue) {
    if (clawRotationPistonValue == false){
      clawRotationPiston.set(DoubleSolenoid.Value.kReverse);
    } else if (clawRotationPistonValue == true){
      clawRotationPiston.set(DoubleSolenoid.Value.kForward);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
