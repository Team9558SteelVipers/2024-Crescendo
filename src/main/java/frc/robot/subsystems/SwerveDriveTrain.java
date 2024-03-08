// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.swerveConstants.*;

import frc.robot.OI;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDriveTrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  private XboxController driveController = new OI().getDriverController();

  final double deadZone = 0.05;

  int navXDebug = 0;

  SwerveModulePosition[] swerveModulePosition;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final SwerveModule m_frontLeft = 
      new SwerveModule(
          frontLeftDrivePort,
          frontLeftRotatePort,
          false);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          frontRightDrivePort,
          frontRightRotatePort,
          false);

  private final SwerveModule m_backLeft = 
      new SwerveModule(
          backLeftDrivePort,
          backLeftRotatePort,
          false);

  private final SwerveModule m_backRight = 
      new SwerveModule(
          backRightDrivePort,
          backRightRotatePort,
          false);

  

    private final Pigeon2 m_pigeon;

  private final SwerveDriveOdometry m_odometry;

  public SwerveDriveTrain() {
    m_odometry =
      new SwerveDriveOdometry(kinematics, new Rotation2d(0), 
      new SwerveModulePosition[]{new SwerveModulePosition(m_frontLeft.getMotorVelocity(), new Rotation2d(m_frontLeft.getTurningRadians())), 
            new SwerveModulePosition(m_frontRight.getMotorVelocity(), new Rotation2d(m_frontRight.getTurningRadians())),
            new SwerveModulePosition(m_backLeft.getMotorVelocity(), new Rotation2d(m_backLeft.getTurningRadians())),
            new SwerveModulePosition(m_backLeft.getMotorVelocity(), new Rotation2d(m_backLeft.getTurningRadians()))});
    m_pigeon = new Pigeon2(pigeonID);
    m_pigeon.reset();

    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelativeSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Deadzones
    if (Math.abs(xSpeed) <= deadZone)
    xSpeed = 0;
    if (Math.abs(ySpeed) <= deadZone)
    ySpeed = 0;
    if (Math.abs(rot) <= deadZone)
    rot = 0;  

    // Try to normalize joystick limits to speed limits
    //xSpeed *= Math.abs(xSpeed);
    //ySpeed *= Math.abs(ySpeed);      
    //rot *= Math.abs(rot);

    var swerveModuleStates = kinematics.toSwerveModuleStates(fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_pigeon.getRotation2d())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    /*
    SmartDashboard.putNumber("Front Left Drive Desired State", swerveModuleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Front Left Turn Desired State", swerveModuleStates[0].angle.getDegrees());

    SmartDashboard.putNumber("Front Right Drive Desired State", swerveModuleStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right Turn Desired State", swerveModuleStates[1].angle.getDegrees());
    
    SmartDashboard.putNumber("Back Left Drive Desired State", swerveModuleStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("Back Left Turn Desired State", swerveModuleStates[2].angle.getDegrees());

    SmartDashboard.putNumber("Back Right Drive Desired State", swerveModuleStates[3].speedMetersPerSecond);
    SmartDashboard.putNumber("Back Right Turn Desired State", swerveModuleStates[3].angle.getDegrees());
    */

    SmartDashboard.putNumber("Front Left Drive Encoder Values", m_frontLeft.getEncoderDrive());
    SmartDashboard.putNumber("Front Left Turn Encoder Values", m_frontLeft.getTurnAngle());

    SmartDashboard.putNumber("Front Right Drive Encoder Values", m_frontRight.getEncoderDrive());
    SmartDashboard.putNumber("Front Right Turn Encoder Values", m_frontRight.getTurnAngle());
    
    SmartDashboard.putNumber("Back Left Drive Encoder Values", m_backLeft.getEncoderDrive());
    SmartDashboard.putNumber("Back Left Turn Encoder Values", m_backLeft.getTurnAngle());

    SmartDashboard.putNumber("Back Right Drive Encoder Values", m_backRight.getEncoderDrive());
    SmartDashboard.putNumber("Back Right Turn Encoder Values", m_backRight.getTurnAngle());

    SmartDashboard.putNumber("Pigeon Reading", m_pigeon.getRotation2d().getDegrees());

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  //**Drives the robot given speeds relative to the robot. */

  public void driveRobotRelativeSpeeds(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  //**Returns the Chassis Speeds relative to the robot. */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    SwerveModuleState[] swerveModuleStates = {m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState()};
    return kinematics.toChassisSpeeds(swerveModuleStates);
  }


  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
        m_odometry.update(
        m_pigeon.getRotation2d(),
        new SwerveModulePosition[]{m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()});
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_pigeon.getRotation2d(), swerveModulePosition, pose);

    m_frontLeft.setPose(pose);
    m_frontRight.setPose(pose);
    m_backLeft.setPose(pose);
    m_backRight.setPose(pose);

    /*for(int i = 0; i < mSwerveModules.length; i++) {
        mSwerveModules[i].setPose(pose);
        mSwerveModules[i].resetEncoders();
    }*/
  }
public Pose2d getPose() {
  return m_odometry.getPoseMeters();
  }
  public Pose2d[] getModulePoses() {
    Pose2d[] modulePoses = {
        m_frontLeft.getPose(),
        m_frontRight.getPose(),
        m_backRight.getPose(),
        m_backLeft.getPose()
    };
    return modulePoses;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    updateSmartDashboard();
  }

  public void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(driveController.getLeftY())
            * maxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(driveController.getLeftX())
            * maxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(driveController.getRightX())
            * maxRotation;

    SmartDashboard.putNumber("XSpeed", xSpeed);
    SmartDashboard.putNumber("YSpeed", ySpeed);
    SmartDashboard.putNumber("Rot", rot);


    drive(xSpeed, ySpeed, rot, fieldRelative);
  }

      /**
     * Returns the raw angle of the robot in degrees
     *
     * @return The angle of the robot
     */
  public double getRawGyroAngle() {
    return m_pigeon.getRotation2d().getDegrees();
  }

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Robot Angle",getRawGyroAngle());
    SmartDashboard.putNumber("Front Left "  + " Angle", m_frontLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("Front Left" + " Speed", m_frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right"  + " Angle", m_frontRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("Front Right" + " Speed", m_frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Back Left"  + " Angle", m_backLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("Back Left" + " Speed", m_backLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Back Right"  + " Angle", m_backRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("BackRight" + " Speed", m_backRight.getState().speedMetersPerSecond);
  }
    /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

}