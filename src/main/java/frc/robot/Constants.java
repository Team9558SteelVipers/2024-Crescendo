package frc.robot;

public final class Constants {
  
  public static class ElevatorConstants {
    // The elevator heights will contain the encoder values for each of the heights
    final public static int clawElevatorPort = 23;
    final public static double[] elevatorHeights = {0,0,0};
    final public static int[] ratchetPistonPort = {1,2,3};
    final public static double P = 1.0;
    final public static double I = 0.0;
    final public static double D = 0.0;
  }

  public static class IntakeConstants {
    final public static int intakeMotorPort = 21;
    final public static double intakeMotorSpeed = 0.25;
    final public static double timerLength = 2;
    final public static double clawIntakeSpeed = 0.25;
    final public static double intakeDistanceConstant = 0;

  }

  public static class ScoringConstants {
    final public static int scoringMotorPort = 22;
    final public static int beamBreakMotorPort = 0;
    // Circumference / Tick
    final public static int EnMotorRatio = 0/4096;
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
