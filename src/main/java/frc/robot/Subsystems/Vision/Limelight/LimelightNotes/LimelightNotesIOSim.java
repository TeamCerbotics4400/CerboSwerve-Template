package frc.robot.Subsystems.Vision.Limelight.LimelightNotes;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class LimelightNotesIOSim implements LimelightNotesIO {
  private double x = 0;
  private double y = 0;
  private final List<Translation3d> notePositions =
      List.of(
          new Translation3d(2.9, 7.0, 0.025),
          new Translation3d(2.9, 5.55, 0.025),
          new Translation3d(2.9, 4.1, 0.025),
          new Translation3d(8.29, 7.44, 0.025),
          new Translation3d(8.29, 5.78, 0.025),
          new Translation3d(8.29, 4.09, 0.025),
          new Translation3d(8.29, 2.44, 0.025),
          new Translation3d(8.29, 0.77, 0.025),
          new Translation3d(13.67, 4.10, 0.025),
          new Translation3d(13.67, 5.55, 0.025),
          new Translation3d(13.67, 7.0, 0.025),
          new Translation3d(15.5, 0.75, 0.025),
          new Translation3d(1.04, 0.75, 0.025));

  @Override
  public void updateInputs(LimelightNotesInputs inputs) {
    Pose3d robotPose = new Pose3d(RobotContainer.getSwerveSubsystem().getPose());
    Pose3d llPose =
        robotPose.transformBy(
            new Transform3d(
                Constants.VisionConstants.noteCam.getTranslation(),
                Constants.VisionConstants.noteCam.getRotation()));

    Translation3d target = null;
    for (Translation3d pos : notePositions) {
      Rotation2d angleToObj =
          pos.toTranslation2d().minus(llPose.getTranslation().toTranslation2d()).getAngle();
      Rotation2d diff = llPose.getRotation().toRotation2d().minus(angleToObj);

      if (Math.abs(diff.getDegrees()) <= (63.3 / 2.0)) {
        double distance = llPose.getTranslation().getDistance(pos);
        if (distance <= 6.0 && target == null) {
          target = pos;

        } else if (target != null && distance < llPose.getTranslation().getDistance(target)) {
          target = pos;
        }
        if (target == null) {

          x = 0;
          y = 0;
        } else {
          x = target.getX();
          y = target.getY();
        }
        Logger.recordOutput("Vision/Testx", x);
        Logger.recordOutput("Vision/Testy", y);
      }
    }

    if (target != null) {
      inputs.hasTarget = true;
      inputs.timestamp = Timer.getFPGATimestamp();

      Rotation2d angleToObj =
          target.toTranslation2d().minus(llPose.getTranslation().toTranslation2d()).getAngle();
      Rotation2d diff = llPose.getRotation().toRotation2d().minus(angleToObj);
      inputs.tx = diff.getDegrees();

      double h = llPose.getTranslation().getDistance(target);
      double o = llPose.getZ() - target.getZ();
      inputs.ty = -Units.radiansToDegrees(Math.asin(o / h) - llPose.getRotation().getY());

      double distance =
          Math.sqrt(
              Math.pow(llPose.getX() - target.getX(), 2)
                  + Math.pow(llPose.getY() - target.getY(), 2));

      inputs.distanceFromTarget = distance;
    } else {
      inputs.hasTarget = false;
      inputs.distanceFromTarget = 0;
    }

    inputs.fps = 30.0;
    inputs.lastFPSTimestamp = Timer.getFPGATimestamp();
    inputs.currentNotePose = new Pose2d(x, y, new Rotation2d());
  }
}
