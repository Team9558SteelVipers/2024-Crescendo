package frc.robot;


import static frc.robot.Constants.oiConstants.*;

import edu.wpi.first.wpilibj.XboxController;


public class OI {

   static XboxController driverController;
    static XboxController operatorController;

    public OI(){
        driverController = new XboxController(driverControllerPort);
        operatorController = new XboxController(operatorControllerPort);
        
        
    }

    public XboxController getDriverController() {
        return driverController;
    }

    public XboxController getOperatorController() {
        return operatorController;
    }
}



//This is a test change