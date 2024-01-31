package frc.robot;


import static frc.robot.Constants.oiConstants.*;

import edu.wpi.first.wpilibj.XboxController;

//A = 1
//B = 2
//X = 3
//Y = 4
//This class will handle all button inputs
public class OI {

   static XboxController driverController;
    static XboxController operatorController;

    public OI(){
        driverController = new XboxController(driverControllerPort);
        operatorController = new XboxController(operatorControllerPort);
        
        
    }

    //////////////////////////////////////DRIVER INPUTS//////////////////////////////////////

    ///////////////////Joystick///////////////////
    //Analog Inputs

    public double getDLeftJoystickX() {
        return driverController.getLeftX();
    }
    
    public double getDLeftJoystickY() {
        return driverController.getLeftY();
    } 

    public double getDRightJoystickX() {
        return driverController.getLeftX();
    } 

    public double getDRightJoystickY() {
        return driverController.getLeftY();
    } 

    ///////////////////Triggers///////////////////
    //Analog Inputs

    public double getDRightTrigger() {
        return driverController.getRightTriggerAxis();
    }

    public double getDLeftTrigger() {
        return driverController.getLeftTriggerAxis();
    }

    ///////////////////Bumpers///////////////////

    public boolean getDRightBumper() {
        return driverController.getRawButton(6);
    }

    public boolean getDLeftBumper() {
        return driverController.getRawButton(5);
    }

    ///////////////////Buttons///////////////////

    public boolean getDAButton() {
        return driverController.getRawButton(1);
    }

    public boolean getDBButton() {
        return driverController.getRawButton(2);
    }

    public boolean getDXButton() {
        return driverController.getRawButton(3);
    }

    public boolean getDYButton() {
        return driverController.getRawButton(4);
    }

    //////////////////////////////////////OPERATOR INPUTS//////////////////////////////////////

    ///////////////////Joystick///////////////////
    //Analog Inputs

    public double getOLeftJoystickX() {
        return operatorController.getLeftX();
    }
    
    public double getOLeftJoystickY() {
        return operatorController.getLeftY();
    } 

    public double getORightJoystickX() {
        return operatorController.getLeftX();
    } 

    public double getORightJoystickY() {
        return operatorController.getLeftY();
    } 

    ///////////////////Triggers///////////////////
    //Analog Inputs

    public double getORightTrigger() {
        return operatorController.getRightTriggerAxis();
    }

    public double getOLeftTrigger() {
        return operatorController.getLeftTriggerAxis();
    }

    ///////////////////Bumpers///////////////////

    public boolean getORightBumper() {
        return operatorController.getRawButton(6);
    }

    public boolean getOLeftBumper() {
        return operatorController.getRawButton(5);
    }

    ///////////////////Buttons///////////////////

    public boolean getOAButton() {
        return operatorController.getRawButton(1);
    }

    public boolean getOBButton() {
        return operatorController.getRawButton(2);
    }

    public boolean getOXButton() {
        return operatorController.getRawButton(3);
    }

    public boolean getOYButton() {
        return operatorController.getRawButton(4);
    }

}
