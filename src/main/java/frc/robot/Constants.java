package frc.robot;

public final class Constants {

  final public static int PCM_ID = 30;

  final public static double kStatorCurrentLimit = 30.0; // TODO: This is generally a good start. Double check on phoenix tuner the current required to move your elevator
  
  public static class ElevatorConstants {

    // The elevator heights will contain the encoder values for each of the heights
    final public static int clawElevatorPort = 23;
    final public static double[] elevatorHeights = {-0.157617,-20.81641};
    final public static double maxHeight = 22;
    final public static double minHeight = 0;
    final public static double maxElevatorSpeed = 0.5;
    final public static double maxReverseElevatorSpeed = -0.5;

    
    public static final double kElevatorP = 0.35;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0.0;
    public static final double kElevatorG = 0.3; //motor output required to hold elevator at constant height, fights gravity
  }

  public static class ClimberConstants {
    final public static int climberMotorPort = 20;
    final public static int[] ratchetPistonPort = {2,5};
    final public static double maxSpeed = 0.3;
  }

  public static class IntakeConstants {
    final public static int intakeMotorPort = 21;
    final public static double intakeMotorSpeed = 0.50;
    final public static double timerLength = 2;
    final public static double clawIntakeSpeed = 0.50;
    final public static double intakeDistanceConstant = 0;
  }

  public static class ScoringConstants {
    final public static int scoringMotorPort = 22;
    final public static int beamBreakMotorPort = 0;
    // Circumference / Tick
    final public static int[] ClawPistonPorts = {3,4};
    final public static double motorScoringSpeed = -0.75;
  }

  public static class oiConstants {
    
    final public static int driverControllerPort = 0;
    final public static int operatorControllerPort = 1;
    
    }

  ///////////////////////////////////////TRAP OPTIMIZATION CONSTANTS/////////////////////////////////////////
// 0 = left red
// 1 = right red
// 2 = far red
// 3 = left blue
// 4 = right blue
// 5 = far blue
  public static class TrapPositions {
    final public static double[] TrapX = {0,0,0,0,0,0};
    final public static double[] TrapY = {0,0,0,0,0,0};
  }


}
