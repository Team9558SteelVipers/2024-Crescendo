package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;


public class IntakeSubsystem extends SubsystemBase {
  
  static TalonFX intakeMotor;
  
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(intakeMotorPort);
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
