package frc.robot.Subsystems.Vision.Limelight.LimelightNotes;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class LimelightNotes extends SubsystemBase {
  private final LimelightNotesIO io;
  private final LimelightNotesInputsAutoLogged inputs;
  /*private Optional<Translation2d> noteLocation;
  private static final double LIMELIGHT_HEIGHT = 25.1; // todo get real num*/

  public LimelightNotes(LimelightNotesIO io) {
    this.io = io;
    this.inputs = new LimelightNotesInputsAutoLogged();
  }

  @Override
  public void periodic() {
    long startTime = Logger.getRealTimestamp();

    if (!Logger.hasReplaySource()) {
      io.updateInputs(inputs);
    }

    Logger.processInputs("LimelightNotes", inputs);

    if (hasTarget()) {
      Logger.recordOutput(
          "LimelightNotes/EstimatedFieldPos", new Pose3d(getEstimatedFieldPos(), new Rotation3d()));
    } else {
      Logger.recordOutput("LimelightNotes/EstimatedFieldPos", new Pose3d());
    }

    double runtimeMS = (Logger.getRealTimestamp() - startTime) / 1000.0;
    Logger.recordOutput("LimelightNotes/PeriodicRuntimeMS", runtimeMS);

    /*if (hasTarget()) {
      double d = LIMELIGHT_HEIGHT / Math.tan(Units.degreesToRadians(inputs.ty + 0));
      double x = d * Math.cos(Units.degreesToRadians(inputs.tx));
      double y = d * Math.sin(Units.degreesToRadians(inputs.tx));
      SmartDashboard.putNumber("Note detection x", x);
      SmartDashboard.putNumber("Note detection y", y);
      noteLocation = Optional.of(new Translation2d(x, y));
    } else {
      noteLocation = Optional.empty();
    }

    notePose().ifPresent(pose -> Logger.recordOutput("CalculatedNotePosition", pose.getX()));*/
  }

  // public Optional<Translation2d> notePose() {
  // return noteLocation;
  // }

  public boolean hasTarget() {
    return inputs.hasTarget && (Timer.getFPGATimestamp() - inputs.lastFPSTimestamp < 5.0);
  }

  public Rotation2d getAngleToTarget() {
    return Rotation2d.fromDegrees(-inputs.tx);
  }

  public Rotation2d getTY() {
    return Rotation2d.fromDegrees(inputs.ty);
  }

  public Pose2d notePosition() {
    return inputs.currentNotePose;
  }

  public double getMeasurementTimestamp() {
    return inputs.timestamp;
  }

  public Translation3d getEstimatedFieldPos() {
    Pose3d robotPose = new Pose3d(RobotContainer.getSwerveSubsystem().getPose());
    Pose3d llPose =
        robotPose.transformBy(
            new Transform3d(
                Constants.VisionConstants.noteCam.getTranslation(),
                Constants.VisionConstants.noteCam.getRotation()));
    Logger.recordOutput("LLPose", llPose);

    double estDistance =
        Math.abs(
            (llPose.getZ() - 0.025)
                / Math.sin(Units.degreesToRadians(inputs.ty) - llPose.getRotation().getY()));
    Translation3d llRelative =
        new Translation3d(
            estDistance,
            new Rotation3d(
                0.0, Units.degreesToRadians(-inputs.ty), Units.degreesToRadians(-inputs.tx)));

    Translation3d fieldRelative =
        llRelative.rotateBy(llPose.getRotation()).plus(llPose.getTranslation());
    return new Translation3d(fieldRelative.getX(), fieldRelative.getY(), 0.025);
  }

  public double getDistanceFromTarget() {
    return inputs.distanceFromTarget;
  }

  public double getTx() {
    return inputs.tx;
  }
}
