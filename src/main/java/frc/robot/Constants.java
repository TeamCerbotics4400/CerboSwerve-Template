package frc.robot;

import static frc.robot.Constants.DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static final Mode currentMode = Mode.SIM;
  public static final boolean needToLog = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class DriveConstants {

    public static final double kDriveGearRatio = 5.5;
    public static final double kTurnGearRatio = 10.29;

    // Distance between left and right wheels
    public static final double kTrackWidth = 0.6096;
    // Distance between front and back wheels
    public static final double kWheelBase = 0.635;

    public static final double MaxAngularRate = 1.5 * Math.PI;
    public static final double MaxLinearSpeed = 18.01;

    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = Units.feetToMeters(18.01) * 0.80;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond =
        kPhysicalMaxAngularSpeedRadiansPerSecond / 3;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double traslationP = 5.0,
        traslationD = 0.0,
        rotationP = 5.0,
        rotationD = 0.0;

    public static final String CANBUS_STRING = "Swerve_Canivore";
    public static final int PIGEON_ID = 15;
  }

  public static class AutoConstants {
    public static String autoValue = "2";

    /*public static final PathConstraints kPathConstraints =
    new PathConstraints(
        Units.feetToMeters(MaxLinearSpeed),
        kTeleDriveMaxSpeedMetersPerSecond,
        kPhysicalMaxAngularSpeedRadiansPerSecond,
        Math.PI * 2);*/
  }

  public class FieldConstants {
    public static double fieldLength = Units.inchesToMeters(651.223);
    public static double fieldWidth = Units.inchesToMeters(323.277);
    public static double wingX = Units.inchesToMeters(229.201);
    public static double podiumX = Units.inchesToMeters(126.75);
    public static double startingLineX = Units.inchesToMeters(74.111);

    /* For auto aligning */
    public static Pose2d blueAmpPose = new Pose2d(2.0, 7.62, Rotation2d.fromDegrees(90));
    public static Pose2d bluePickupPose = new Pose2d(15.331, 1, Rotation2d.fromDegrees(-60));
    public static Pose2d redPickupPose = new Pose2d(2, 1, Rotation2d.fromDegrees(-60));
    public static Pose2d redAmpPose = new Pose2d(15.331, 7.62, Rotation2d.fromDegrees(90));

    /*For note simulation */
    public static final Translation3d blueSpeaker = new Translation3d(0.225, 5.55, 2.1);
    public static final Translation3d redSpeaker = new Translation3d(16.317, 5.55, 2.1);
    public static final Translation3d blueAmp = new Translation3d(1.85, 8.25, 0.8);
    public static final Translation3d redAmp = new Translation3d(15.333, 8.25, 0.8);

    /** Staging locations for each note */
    public static final class StagingLocations {
      public static final double centerlineX = fieldLength / 2.0;

      public static final double centerlineFirstY = Units.inchesToMeters(29.638);
      public static final double centerlineSeparationY = Units.inchesToMeters(66);
      public static final double spikeX = Units.inchesToMeters(114);

      public static final double spikeFirstY = Units.inchesToMeters(161.638);
      public static final double spikeSeparationY = Units.inchesToMeters(57);

      public static final Translation2d[] centerlineTranslations = new Translation2d[5];
      public static final Translation2d[] spikeTranslations = new Translation2d[3];

      static {
        for (int i = 0; i < centerlineTranslations.length; i++) {
          centerlineTranslations[i] =
              new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
        }
      }

      static {
        for (int i = 0; i < spikeTranslations.length; i++) {
          spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
        }
      }
    }
  }

  public static class ModuleConstants {
    /*
     *                   F
     *   ┌───────┬─────────────────┬───────┐
     *   │       │                 │       │
     *   │ Mod 0 │                 │ Mod 1 │
     *   │       │                 │       │
     *   ├───────┘                 └───────┤
     *   │                                 │
     *   │            Modules              │
     * L │            Diagram              │ R
     *   │                                 │
     *   │                                 │
     *   │                                 │
     *   ├───────┐                 ┌───────┤
     *   │       │                 │       │
     *   │ Mod 3 │                 │ Mod 2 │
     *   │       │      arm        │       │
     *   └───────┴─────────────────┴───────┘
     *                  B
     */

    /** Front Left module 0 * */
    public static final byte kFrontLeftDriveMotorId = 1;

    public static final byte kFrontLeftSteerMotorId = 2;
    public static final byte kFrontLeftEncoderId = 3;
    public static final double kFrontLeftEncoderOffset = -2.15;
    /** Front Right module 1 * */
    public static final byte kFrontRightDriveMotorId = 4;

    public static final byte kFrontRightSteerMotorId = 5;
    public static final byte kFrontRightEncoderId = 6;
    public static final double kFrontRightEncoderOffset = -2.9;
    /** Back Left module 2 * */
    public static final byte kBackLeftDriveMotorId = 10;

    public static final byte kBackLeftSteerMotorId = 11;
    public static final byte kBackLeftEncoderId = 12;
    public static final double kBackLeftEncoderOffset = 1.82;
    /** Back Right module 3 * */
    public static final byte kBackRightDriveMotorId = 7;

    public static final byte kBackRightSteerMotorId = 8;
    public static final byte kBackRightEncoderId = 9;
    public static final double kBackRightEncoderOffset = -2.085;

    /** To change offsets easily * */
    // Minus  = Counterclockwise
    // Plus = Clockwise
  }

  public static final class VisionConstants {

    public static final String neuralLimelight = "limelight-neural";
    public static final String tagLimelightName = "limelight-tags";

    public static final int main_Pipeline = 0,
        upper_Pipeline = 1,
        medium_Pipeline = 2,
        lower_Pipeline = 3;

    public static final double ambiguityThreshold = 0.15;

    public static final String photonCam1 = "Placeholder0";
    public static final String photonCam2 = "Placeholder1";
    public static final String photonCam3 = "Placeholder2";
    public static final String photonCam4 = "Placeholder3";

    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center,
    // values are in meters.
    public static final Transform3d kRobotToCam1 =
        new Transform3d(
            new Translation3d(0.169418, 0.323596, 0.63754),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-25.0),
                Units.degreesToRadians(15.0)));
    public static final Transform3d kRobotToCam2 =
        new Transform3d(
            new Translation3d(0.169418, -0.323596, 0.63754),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-25.0),
                Units.degreesToRadians(-15.0)));
    public static final Transform3d kRobotToCam3 =
        new Transform3d(
            new Translation3d(0.033782, 0.323596, 0.63754),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-25.0),
                Units.degreesToRadians(165.0)));
    public static final Transform3d kRobotToCam4 =
        new Transform3d(
            new Translation3d(0.033782, -0.323596, 0.63754),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-25.0),
                Units.degreesToRadians(-165.0)));

    public static final Transform3d noteCam =
        new Transform3d(
            new Translation3d(0.0, 0.0, 0.63754),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(0.0)));

    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1500, 1500, 1400);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(1000, 1000, 1400);

    public static double xyStdDevCoefficient = 0.02;
    public static double thetaStdDevCoefficient = 0.04;
  }

  /*   _________
   /   _____/__  __ ____   ____
   \_____  \\  \/ // __ \ /    \
  /_______  /\   /\  ___/|   |  \
          \/  \_/  \___  >___|  /
                       \/     \/
          */
}
