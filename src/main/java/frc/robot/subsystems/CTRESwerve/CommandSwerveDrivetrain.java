package frc.robot.subsystems.CTRESwerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CTRESwerve.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Rotation2d m_headingToMaintain = new Rotation2d();
    private int controlCounter = 0;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        setAzimuthMotorsStatorCurrent();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        setAzimuthMotorsStatorCurrent();
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
                                            
            ()->{
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }
    
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Rotation2d getOperatorForwardDirection()
    {
        return m_operatorForwardDirection;
    }

    public Rotation2d getCurrentRobotHeading()
    {
        return getState().Pose.getRotation().minus(m_fieldRelativeOffset);
    }

    public Rotation2d getHeadingToMaintain()
    {
        return m_headingToMaintain;
    }

    public void setHeadingToMaintain(Rotation2d newHeading)
    {
        m_headingToMaintain = newHeading;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void setAzimuthMotorsStatorCurrent()
    {
        CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs().withStatorCurrentLimit(150).withStatorCurrentLimitEnable(true);

        getModule(0).getSteerMotor().getConfigurator().apply(currentLimit);
        getModule(1).getSteerMotor().getConfigurator().apply(currentLimit);
        getModule(2).getSteerMotor().getConfigurator().apply(currentLimit);
        getModule(3).getSteerMotor().getConfigurator().apply(currentLimit);
    }

    public Command applyRequest(CommandXboxController driverController, Supplier<SwerveRequest> requestSupplierWithControl, Supplier<SwerveRequest> requestSupplierWithHeading) {
        return run(() -> {
            Supplier<SwerveRequest> activeRequest = requestSupplierWithControl;

            if (Math.abs(driverController.getRightX()) > 0.08)
            {
                controlCounter = 5;
            }
            else
            {
                if (controlCounter > 0){
                    m_headingToMaintain = getCurrentRobotHeading();
                    controlCounter--;
                }
                else
                {
                    if (Math.abs(m_headingToMaintain.getRadians() - getCurrentRobotHeading().getRadians()) > 6)
                    {
                        m_headingToMaintain = getCurrentRobotHeading();
                    } else {/* do nothing */}
                    activeRequest = requestSupplierWithHeading;
                }
            }

            this.setControl(activeRequest.get());
        });

    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}