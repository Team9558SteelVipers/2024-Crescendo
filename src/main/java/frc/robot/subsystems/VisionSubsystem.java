package frc.robot.subsystems;

import frc.robot.Constants.MonoCam1;
import frc.robot.Constants.MonoCam2;

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
import java.util.List;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class VisionSubsystem extends Thread {
	boolean useVision = true;
	Alliance lastAlliance = null;
	AprilTagFieldLayout aprilTagFieldLayout;

	public PhotonCamera Mono1;
	Transform3d robotToMono1 = new Transform3d(
            new Translation3d(MonoCam1.xOffset, MonoCam1.yOffset, MonoCam1.zOffset),
			new Rotation3d(Math.toRadians(MonoCam1.rollOffset), Math.toRadians(MonoCam1.pitchOffset), Math.toRadians(MonoCam1.yawOffset)));

	public PhotonCamera Mono2;
	Transform3d robotToMono2 = new Transform3d(
            new Translation3d(MonoCam2.xOffset, MonoCam2.yOffset, MonoCam2.zOffset),
			new Rotation3d(Math.toRadians(MonoCam2.rollOffset), Math.toRadians(MonoCam2.pitchOffset), Math.toRadians(MonoCam2.yawOffset)));

	public PhotonCamera Color1;
	public PhotonCamera Color2;

	PhotonPoseEstimator Mono1PhotonPoseEstimator;
	PhotonPoseEstimator Mono2PhotonPoseEstimator;

	Optional<EstimatedRobotPose> resultMono1;
	Optional<EstimatedRobotPose> resultMono2;
	
	double Mono1LastTimeStamp = 0;
	double Mono2LastTimeStamp = 0;


	public VisionSubsystem() {
		super();
			try {
				aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
			} catch (Exception e) {
				aprilTagFieldLayout = null;
			}

			Mono1 = new PhotonCamera("Mono 1"); //TODO rename to corresponding namae is photonision UI
			Mono1PhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToMono1);

			Mono2 = new PhotonCamera("Mono 2"); //TODO rename to corresponding namae is photonision UI
			Mono2PhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToMono2);

			setVisionWeights(0.2, 0.2, 10);
	}


	public Optional<EstimatedRobotPose> getEstimatedMono1GlobalPose() {
		List<PhotonPipelineResult> r = Mono1.getAllUnreadResults();
		return Mono1PhotonPoseEstimator.update(r.get(r.size() - 1));
	}

	public Optional<EstimatedRobotPose> getEstimatedMono2GlobalPose() {
		List<PhotonPipelineResult> r = Mono1.getAllUnreadResults();
		return Mono2PhotonPoseEstimator.update(r.get(r.size() - 1));
	}

	public void setVisionWeights(double visionX, double visionY, double visionDeg){
		RobotContainer.m_SwerveDriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(visionX, visionY, Units.degreesToRadians(visionDeg)));
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
		Matrix<N3, N1> weights = new Matrix<N3, N1>(new SimpleMatrix(new double[]{distanceRatio * targetMultiplier, distanceRatio * targetMultiplier, 3 + 15 * distanceRatio * targetMultiplier}));
		return weights;
	}

	public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> weights) {
		RobotContainer.m_SwerveDriveTrain.addVisionMeasurement(pose, timestampSeconds, weights);
	}

	public PhotonTrackedTarget getBestFrontNote() { //depending on which camera is placed on forward direction of
		var result = Color1.getLatestResult();		//robot, change the camera "Color1" to "Color2" if color2 is 
		var bestTarget = result.getBestTarget();	//placed on front
		return bestTarget;
	}

	public PhotonTrackedTarget getBestBackNote() {
		var result = Color2.getLatestResult();
		var bestTarget = result.getBestTarget();
		return bestTarget;
	}

	@Override
	public void run() {
		while(true) {
			try{
				Thread.sleep(10);
			} catch (InterruptedException e) {
				//TODO auto-generated catch block
				e.printStackTrace();
			}

			//vision calculations

			if(DriverStation.getAlliance().isPresent()) {
				if(DriverStation.getAlliance().get() == Alliance.Blue && lastAlliance != Alliance.Blue){
					lastAlliance = Alliance.Blue;
					aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
				}
				if(DriverStation.getAlliance().get() == Alliance.Red && lastAlliance != Alliance.Red) {
					lastAlliance = Alliance.Red;
					aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
				}
			}

			Mono1PhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
			Mono2PhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);

			this.resultMono1 = getEstimatedMono1GlobalPose();
			this.resultMono2 = getEstimatedMono2GlobalPose();

			if(useVision) {
				
				if(resultMono1.isPresent()) {
					EstimatedRobotPose camPoseMono1 = resultMono1.get();
					double Mono1TimeStamp = camPoseMono1.timestampSeconds;
					if(Mono1TimeStamp > Timer.getFPGATimestamp()) {
						Mono1TimeStamp = Timer.getFPGATimestamp();
					}

					double sum = 0;
					for(PhotonTrackedTarget target : camPoseMono1.targetsUsed){
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getTranslation().toTranslation2d();
						sum += resultMono1.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
					}
					sum /= camPoseMono1.targetsUsed.size();
					double distanceRatio = sum;
					Matrix<N3, N1> weights = getVisionWeights(distanceRatio, camPoseMono1.targetsUsed.size());

					if(Mono1TimeStamp != Mono1LastTimeStamp) {
						RobotContainer.m_SwerveDriveTrain.addVisionMeasurement(camPoseMono1.estimatedPose.toPose2d(), Mono1TimeStamp, weights);
					}
					Mono1LastTimeStamp = Mono1TimeStamp;
				}


				if(resultMono2.isPresent()) {
					EstimatedRobotPose camPoseMono2 = resultMono2.get();
					double Mono2TimeStamp = camPoseMono2.timestampSeconds;
					if(Mono2TimeStamp > Timer.getFPGATimestamp()) {
						Mono2TimeStamp = Timer.getFPGATimestamp();
					}

					double sum = 0;
					for(PhotonTrackedTarget target : camPoseMono2.targetsUsed){
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getTranslation().toTranslation2d();
						sum += resultMono2.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
					}
					sum /= camPoseMono2.targetsUsed.size();
					double distanceRatio = sum;
					Matrix<N3, N1> weights = getVisionWeights(distanceRatio, camPoseMono2.targetsUsed.size());

					if(Mono2TimeStamp != Mono2LastTimeStamp) {
						RobotContainer.m_SwerveDriveTrain.addVisionMeasurement(camPoseMono2.estimatedPose.toPose2d(), Mono2TimeStamp, weights);
					}
					Mono2LastTimeStamp = Mono2TimeStamp;
				}

			}
		}
	}
}