package frc.robot.Subsystems.Vision.PhotonVision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Swerve.Drive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonAprilTagVision extends SubsystemBase {
  PhotonSim[] cameras;
  Drive m_drive;

  public PhotonAprilTagVision(Drive m_drive, PhotonSim... cameras) {
    this.cameras = cameras;
    this.m_drive = m_drive;
  }

  @Override
  public void periodic() {
    for (int instanceIndex = 0; instanceIndex < cameras.length; instanceIndex++) {
      var visionEst = cameras[instanceIndex].getEstimatedGlobalPose();
      List<Pose3d> tagPose3ds = new ArrayList<>();
      PhotonPipelineResult unprocessedResult = cameras[instanceIndex].getLatestResult();
      final int index = instanceIndex;
      cameras[instanceIndex].ActivateSimParameters(m_drive.getPose(), instanceIndex);

      for (int id : unprocessedResult.getMultiTagResult().fiducialIDsUsed) {
        tagPose3ds.add(VisionConstants.kTagLayout.getTagPose(id).get());
      }

      visionEst.ifPresent(
          est -> {
            var estPose = est.estimatedPose.toPose2d();
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = cameras[index].getEstimationStdDevs(estPose);

            m_drive.addVisionMeasurement(
                est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          });

      Logger.recordOutput("Vision/Tags Used " + instanceIndex, tagPose3ds.size());
      Logger.recordOutput(
          "Vision/Has tags detected " + instanceIndex, cameras[instanceIndex].hasTargets());
      Logger.recordOutput(
          "Vision/Best Target " + instanceIndex, cameras[instanceIndex].getBestTarget());
      Logger.recordOutput("Vision/Best Target " + instanceIndex, cameras[instanceIndex].getArea());
    }
  }
}
