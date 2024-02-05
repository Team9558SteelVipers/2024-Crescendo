package frc.robot;

public final class Constants {
  
  public static class ElevatorConstants {
    // The elevator heights will contain the encoder values for each of the heights
    final public static int clawElevatorPort = 0;
    final public static double[] elevatorHeights = {0,0,0};
    final public static int[] ratchetPistonPort = {1,2,3};
    final public static double P = 0.0;
    final public static double I = 0.0;
    final public static double D = 0.0;
  }

  public static class IntakeConstants {
    final public static int intakeMotorPort = 0;
    final public static double intakeMotorSpeed = 0.25;
    final public static double timerLength = 2;
    final public static double clawIntakeSpeed = 0.25;
    final public static double intakeDistanceConstant = 0;

  }

  public static class ScoringConstants {
    final public static int scoringMotorPort = 0;
    final public static int beamBreakMotorPort = 0;
  }

  public static class oiConstants {
    
    final public static int driverControllerPort = 0;
    final public static int operatorControllerPort = 0;
    
    }




}
