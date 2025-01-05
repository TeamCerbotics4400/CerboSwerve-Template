package frc.robot.Subsystems.Vision.PhotonVision;

import static frc.robot.Constants.VisionConstants.kTagLayout;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.Subsystems.Swerve.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonSim extends SubsystemBase {
  private final PhotonPoseEstimator photonEstimator;
  private final VisionSystemSim visionSim;
  private final TargetModel targetModel = new TargetModel(0.5, 0.25);
  private final PhotonCamera camera;
  private final Transform3d robotToCamera;
  private final PhotonCameraSim cameraSim;
  private final Pose3d targetPose;
  private final VisionTargetSim visionTarget; // Custom April tag
  private final SimCameraProperties cameraProp;
  private double lastEstTimestamp = 0;

  public PhotonSim(int index) {
    switch (index) {
      case 0:
        robotToCamera = VisionConstants.kRobotToCam1;
        break;

      case 1:
        robotToCamera = VisionConstants.kRobotToCam2;
        break;

      case 2:
        robotToCamera = VisionConstants.kRobotToCam3;
        break;

      case 3:
        robotToCamera = VisionConstants.kRobotToCam4;
        break;

      default:
        throw new IllegalArgumentException("Invalid index");
    }
    camera = new PhotonCamera("PhotonCam " + index); // This camera is just for the simulation
    visionSim = new VisionSystemSim("sim " + index);
    cameraProp = new SimCameraProperties();

    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    cameraProp.setCalibError(0.25, 0.08);
    cameraProp.setFPS(20);
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(camera, cameraProp);

    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);

    cameraSim.enableDrawWireframe(true);
    targetPose = new Pose3d(16, 4, 0.4, new Rotation3d(0, 0, Math.PI));
    visionTarget = new VisionTargetSim(targetPose, targetModel);

    visionSim.addAprilTags(kTagLayout);
    visionSim.addCamera(cameraSim, robotToCamera);

    photonEstimator =
        new PhotonPoseEstimator(
            VisionConstants.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            robotToCamera);
  }

  @Override
  public void periodic() {}

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (Robot.isSimulation()) {
      visionEst.ifPresentOrElse(
          est ->
              getSimDebugField()
                  .getObject("VisionEstimation")
                  .setPose(est.estimatedPose.toPose2d()),
          () -> {
            if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
            Logger.recordOutput(
                "Vision/RealEstimation",
                getSimDebugField().getObject("VisionEstimation").getPose());
          });
    }
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {

    var estStdDevs = VisionConstants.kSingleTagStdDevs;
    var targets = getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    return estStdDevs;
  }

  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }

  // Private method that returns the distance to the target
  public Translation2d getCameraToTarget(double yaw, double pitch, double noteHeightMeters) {

    // Define the vector
    double x = 1.0 * Math.tan(Units.degreesToRadians(yaw));
    double y = 1.0 * Math.tan(Units.degreesToRadians(pitch));
    double z = 1.0;
    double norm = Math.sqrt(x * x + y * y + z * z);
    x /= norm;
    y /= norm;
    z /= norm;

    // Rotate the vector by the camera pitch
    double xPrime = x;
    Translation2d yzPrime =
        new Translation2d(y, z).rotateBy(new Rotation2d(robotToCamera.getRotation().getY()));
    double yPrime = yzPrime.getX();
    double zPrime = yzPrime.getY();

    // Solve for the intersection
    double angleToGoalRadians = Math.asin(yPrime);
    double diffHeight = robotToCamera.getZ() - noteHeightMeters;
    double distance = diffHeight / Math.tan(angleToGoalRadians);

    // Returns the distance to the target (in meters)
    return new Translation2d(distance, Rotation2d.fromDegrees(yaw));
  }

  public double getYaw() {
    return getLatestResult().getBestTarget() == null
        ? 0
        : getLatestResult().getBestTarget().getYaw();
  }

  public double getPitch() {
    return getLatestResult().getBestTarget() == null
        ? 0
        : getLatestResult().getBestTarget().getPitch();
  }

  public void measurements(Drive m_drive) {
    var visionEst = getEstimatedGlobalPose();
    visionEst.ifPresent(
        est -> {
          var estPose = est.estimatedPose.toPose2d();
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = getEstimationStdDevs(estPose);

          m_drive.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });
  }

  public void ActivateSimParameters(Pose2d robotPoseMeters, int index) {
    Logger.recordOutput("Vision/Tags", targetPose);
    Logger.recordOutput("Vision/Cameras", robotToCamera);
    Logger.recordOutput("Vision/Estimated" + index, visionSim.getRobotPose());

    visionSim.update(robotPoseMeters);
  }

  public boolean hasTargets() {
    return getLatestResult().hasTargets();
  }

  public double getBestTarget() {
    return getLatestResult().getBestTarget() == null
        ? -1
        : getLatestResult().getBestTarget().getFiducialId();
  }

  public double getArea() {
    return getLatestResult().getBestTarget() == null
        ? -1
        : getLatestResult().getBestTarget().getArea();
  }
}
