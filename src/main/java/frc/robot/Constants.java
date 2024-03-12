package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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
    // Circumference / Tick
    final public static int EnMotorRatio = 0/4096;
  }

  public static class oiConstants {
    
    final public static int driverControllerPort = 0;
    final public static int operatorControllerPort = 0;
    
    }

  public static class swerveConstants {

    final public static double maxRotation = 0;
    final public static double maxSpeed = 0;

    final public static double ksVolts = 0;
    final public static double kvVoltSecondsPerMeter = 0;
    final public static double kaVoltSecondsSquaredPerMeter = 0;
    final public static double kvVoltSecondsPerRadian = 0;
    final public static double kaVoltSecondsSquaredPerRadian = 0;

    
    final public static double driveDistancePerPulse = 0;
    final public static double MagEncoderCPR = 0;
    final public static double turnDistancePerPulse = 0;

    final public static int frontLeftDrivePort = 0;
    final public static int frontLeftRotatePort = 0;

    final public static int frontRightDrivePort = 0;
    final public static int frontRightRotatePort = 0;

    final public static int backLeftDrivePort = 0;
    final public static int backLeftRotatePort = 0;

    final public static int backRightDrivePort = 0;
    final public static int backRightRotatePort = 0;

    final public static double frameSize = 10;
    final public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(frameSize / 2, frameSize / 2),
      new Translation2d(frameSize / 2, -frameSize / 2),
      new Translation2d(-frameSize / 2, frameSize / 2),
      new Translation2d(-frameSize / 2, -frameSize / 2)
    );

    final public static int pigeonID = 0;
  }



}
