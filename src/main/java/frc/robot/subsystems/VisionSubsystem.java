// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.LimelightHelpers;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
 
//RED TRAP OFFSETS//

public double redLeftTrapOffset() {
  return 0;
}

public double redRightTrapOffset() {
  return 0;
}

public double redBackTrapOffset() {
  return 0;
}

//BLUE TRAP OFFSETS//

public double blueLeftTrapOffset() {
  return 0;
}

public double blueRightTrapOffset() {
  return 0;
}

public double blueBackTrapOffset() {
  return 0;
}

}
