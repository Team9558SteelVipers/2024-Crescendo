package frc.robot;


import static frc.robot.Constants.oiConstants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class OI {

    static CommandXboxController driverController;
    static XboxController operatorController;

    public OI(){
        driverController = new CommandXboxController(driverControllerPort);
        operatorController = new XboxController(operatorControllerPort);
        
        
    }

    public CommandXboxController getDriverController() {
        return driverController;
    }

    public XboxController getOperatorController() {
        return operatorController;
    }
}



//This is a test change part 2