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

public redLeftTrapOffset() {

}

public redRightTrapOffset() {
  Math.sqrt( math.pow(Constants.redRightTrapX - odom.getpose.x,2) + math.pow(Constants.redRightTrapY - odom.getpose.y,2))
}

public redBackTrapOffset() {
  
}

//BLUE TRAP OFFSETS//

public blueLeftTrapOffset() {

}

public blueRightTrapOffset() {
  
}

public blueBackTrapOffset() {
  
}

}
