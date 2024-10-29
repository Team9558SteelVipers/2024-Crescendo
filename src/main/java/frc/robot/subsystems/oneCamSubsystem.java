package main.java.frc.robot.subsystems;

import frc.robot.Constants.cam1;
import frc.robot.Constants.cam2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.RobotContainer;
import java.io.IOException;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;


public class VisionSubsystem extends Thread {

	// Vision Variables
	AprilTagFieldLayout aprilTagFieldLayout;

	public PhotonCamera backLeftCam;

	// transform from the robots origin to the camera's position
	Transform3d robotToBackLeftCam = new Transform3d(
            new Translation3d(cam1.xOffset, cam1.yOffset, cam1.zOffset),
			new Rotation3d(Math.toRadians(cam1.rollOffset), Math.toRadians(cam1.pitchOffset), Math.toRadians(cam1.yawOffset)));


	PhotonPoseEstimator backLeftPhotonPoseEstimator;
	Optional<EstimatedRobotPose> resultBackLeft;
	boolean useVision = true;
    Alliance lastAlliance = null;
	double backLeftLastTimeStamp = 0;
	double visionRatio = 10;

	// constructor of VisionSubsystem
	// A constructor is called to initialize an object
	// ex Square s = new Square(10);
	// calls the constructor of the Square class with the parameter "10" 
	public VisionSubsystem() {
		// initialize thread
		super();

		// Load Field Layout w/ known tags
		try {
			aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

		// should only catch IOEXCEPTION, other exceptions not handled and should cause failure
		} catch (IOException e) {
			System.out.println("ERROR Loading April Tag DATA");
			aprilTagFieldLayout = null;
		}

		backLeftCam = new PhotonCamera("BackLeft");
		// PoseStrategy
		// Use all visible tags to compute a single pose estimate on coprocessor. This option needs to be enabled on the PhotonVision web UI as well.
		backLeftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backLeftCam, robotToBackLeftCam);

		setVisionWeights(.2, .2, 10);
	}

	// Vision Methods

	public Optional<EstimatedRobotPose> getEstimatedBackLeftGlobalPose() {
		return backLeftPhotonPoseEstimator.update();
	}

	public void useVision(boolean useVision) {
		this.useVision = useVision;
	}

	public void setVisionWeights(double visionX, double visionY, double visionDeg) {
		RobotContainer.drivetrainSubsystem
				.setVisionMeasurementStdDevs(VecBuilder.fill(visionX, visionY, Units.degreesToRadians(visionDeg)));
	}

	public static void publishPose2d(String key, Pose2d pose) {
		SmartDashboard.putNumberArray(key, new double[]{pose.getTranslation().getX(), pose.getTranslation().getY(),
				pose.getRotation().getRadians()});
	}

	public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> weights) {
		RobotContainer.drivetrainSubsystem.addVisionMeasurement(pose, timestampSeconds, weights);
	}

	public void log() {
		SmartDashboard.putBoolean("/Vision/BackLeft/Connected", backLeftCam.isConnected());
	}
    
	public Matrix<N3, N1> getVisionWeights(double distanceRatio, int numTargets) {
		double targetMultiplier = 1;
		double visionCutOffDistance = 4;
		distanceRatio = 0.1466 * Math.pow(1.6903, distanceRatio);
		if (numTargets == 1) {
			if (distanceRatio > visionCutOffDistance) {
				return new Matrix<N3, N1>(new SimpleMatrix(new double[]{99999, 99999, 99999}));
			}
			targetMultiplier = 3;
		}
		Matrix<N3, N1> weights = new Matrix<N3, N1>(new SimpleMatrix(new double[]{distanceRatio * targetMultiplier,
				distanceRatio * targetMultiplier, 3 + 15 * distanceRatio * targetMultiplier}));
		return weights;
	}
	
	@Override
	public void run() {
		/* Run as fast as possible, our signals will control the timing */
		while (true) {
			// take 10ms wait at beginning of every loop
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			// Vision Calculations
			if (DriverStation.getAlliance().isPresent()) {
				if (DriverStation.getAlliance().get() == Alliance.Blue && lastAlliance != Alliance.Blue) {
					lastAlliance = Alliance.Blue;
					aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
				}
				if (DriverStation.getAlliance().get() == Alliance.Red && lastAlliance != Alliance.Red) {

					lastAlliance = Alliance.Red;
					aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
				}
			}

			backLeftPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);

			this.resultBackLeft = getEstimatedBackLeftGlobalPose();

			if (useVision) {

				if (resultBackLeft.isPresent()) {
					EstimatedRobotPose camPoseBackLeft = resultBackLeft.get();
					double backLeftTimeStamp = camPoseBackLeft.timestampSeconds;
					if (backLeftTimeStamp > Timer.getFPGATimestamp()) {
						backLeftTimeStamp = Timer.getFPGATimestamp();
					}

					double sum = 0;

					// for every april tag
					for (PhotonTrackedTarget target : camPoseBackLeft.targetsUsed) {
						// get position of tag
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
								.getTranslation().toTranslation2d();
						// add the distance to sum
						sum += resultBackLeft.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
					}
					// divide by all targets used to get average distance to a tag
					sum /= camPoseBackLeft.targetsUsed.size();
					double distanceRatio = sum;
					Matrix<N3, N1> weights = getVisionWeights(distanceRatio, camPoseBackLeft.targetsUsed.size());

					// if new data
					if (backLeftTimeStamp != backLeftLastTimeStamp) {
						// log robot estimated position
						publishPose2d("/DriveTrain/BackLeftCamPose", camPoseBackLeft.estimatedPose.toPose2d());
						// log robot weights
						SmartDashboard.putString("/Vision/BackLeftWeights", weights.toString());
						// add location to drive system
						RobotContainer.drivetrainSubsystem.addVisionMeasurement(
								camPoseBackLeft.estimatedPose.toPose2d(), backLeftTimeStamp, weights);
					}
					// update last time stamp
					backLeftLastTimeStamp = backLeftTimeStamp;
				}

			}
		}
	}
}
