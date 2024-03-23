package frc.robot;


import static frc.robot.Constants.oiConstants.*;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class OI {

    static CommandXboxController driverController;
    static CommandXboxController operatorController;

    public OI(){
        driverController = new CommandXboxController(driverControllerPort);
        operatorController = new CommandXboxController(operatorControllerPort);
        
        
    }

    public CommandXboxController getDriverController() {
        return driverController;
    }

    public CommandXboxController getOperatorController() {
        return operatorController;
    }
}



//This is a test change part 2