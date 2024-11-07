// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.oneCamSubsystem;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface oneCamIO {

    @AutoLog
    class oneCameIOInputs{
        boolean cameraConnected = false;
        String pipeline = "";
        int aprilTagsDetected = 0;
        //will order from most confident to least confident
        double[] aprilTagX = {};
        double[] aprilTagy = {};


        
    }
} 
