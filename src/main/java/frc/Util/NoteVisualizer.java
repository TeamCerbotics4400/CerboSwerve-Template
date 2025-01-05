// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer {

  private static final Transform3d launcherTransform =
      new Transform3d(-0.1, 0, 1.0, new Rotation3d(0.0, Units.degreesToRadians(-55.0), 0.0));
  private static final double shotSpeed = 9.0; // Meters per sec
  private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();
  private static boolean hasNote = false;
  private static final List<Translation2d> autoNotes = new ArrayList<>();
  private static Pose3d note = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  private static double getZ = 0.0;

  public static void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
    robotPoseSupplier = supplier;
  }

  /** Show all staged notes for alliance */
  public static void showAutoNotes() {
    if (autoNotes.isEmpty()) {
      Logger.recordOutput("NoteVisualizer/StagedNotes", new Pose3d[] {});
    }
    // Show auto notes
    Stream<Translation2d> presentNotes = autoNotes.stream().filter(Objects::nonNull);
    Logger.recordOutput(
        "NoteVisualizer/StagedNotes",
        presentNotes
            .map(
                translation ->
                    new Pose3d(
                        translation.getX(),
                        translation.getY(),
                        Units.inchesToMeters(1.0),
                        new Rotation3d()))
            .toArray(Pose3d[]::new));
  }

  public static void teleopNote() {
    final boolean isBlue =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get().equals(Alliance.Blue);
    if (!hasSimNote()) {
      note =
          isBlue
              ? new Pose3d(15.331, 1.0, 0.0254, new Rotation3d(0.0, 0, 0.0))
              : new Pose3d(1, 1.02, 0.0254, new Rotation3d(0, 0, 0));
    } else {
      note = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    }

    Logger.recordOutput("NoteVisualizer/TeleopNote", note);
  }

  public static Pose2d getSourceNote() {
    return note.toPose2d();
  }

  public static void clearAutoNotes() {
    autoNotes.clear();
  }

  /**
   * Take note from staged note
   *
   * @param note Number of note starting with 0 - 2 being spike notes going from amp to source side
   *     <br>
   *     and 3 - 7 being centerline notes going from amp to source side.
   */
  public static void takeAutoNote(int note) {
    autoNotes.set(note, null);
    hasNote = true;
  }

  public static void deleteNote(int note) {
    autoNotes.set(note, null);
  }

  public static void enableShowNote() {
    hasNote = true;
  }

  public static void disableShowNote() {
    hasNote = false;
  }

  public static Boolean hasSimNote() {
    return hasNote;
  }

  /** Add all notes to be shown at the beginning of auto */
  public static void resetAutoNotes() {
    clearAutoNotes();
    final boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get().equals(Alliance.Red);

    for (int i = FieldConstants.StagingLocations.spikeTranslations.length - 1; i >= 0; i--) {
      autoNotes.add(
          isRed
              ? new Translation2d(
                  FieldConstants.StagingLocations.spikeTranslations[i].getX(),
                  FieldConstants.StagingLocations.spikeTranslations[i].getY())
              : FieldConstants.StagingLocations.spikeTranslations[i]);
    }
    for (int i = FieldConstants.StagingLocations.centerlineTranslations.length - 1; i >= 0; i--) {
      autoNotes.add(
          isRed
              ? new Translation2d(
                  FieldConstants.StagingLocations.centerlineTranslations[i].getX(),
                  FieldConstants.StagingLocations.centerlineTranslations[i].getY())
              : FieldConstants.StagingLocations.centerlineTranslations[i]);
    }
  }

  public static void sourceNote() {
    final boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get().equals(Alliance.Red);
    autoNotes.add(
        isRed
            ? new Translation2d(
                FieldConstants.redPickupPose.getX(), FieldConstants.redPickupPose.getY())
            : new Translation2d(
                FieldConstants.bluePickupPose.getX(), FieldConstants.bluePickupPose.getY()));
  }

  public static Translation2d getAutoNote(int note) {
    return autoNotes.get(note);
  }

  /** Shows the currently held note if there is one */
  public static void showIntakedNotes(double angleRads) {
    if (hasNote) {
      Logger.recordOutput("NoteVisualizer/HeldNotes", getIndexerPose3d(angleRads));
    } else {
      Logger.recordOutput("NoteVisualizer/HeldNotes", new Pose3d());
    }
  }

  public static Command speakerShoot() {
    return new ScheduleCommand(
        Commands.defer(
                () -> {
                  final Pose3d startPose =
                      new Pose3d(robotPoseSupplier.get()).transformBy(launcherTransform);
                  final boolean isRed =
                      DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Red);
                  final Pose3d endPose =
                      new Pose3d(
                          isRed ? FieldConstants.redSpeaker : FieldConstants.blueSpeaker,
                          startPose.getRotation());

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / shotSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  hasNote = false;
                  return Commands.run(
                          () -> {
                            Logger.recordOutput(
                                "NoteVisualizer",
                                new Pose3d[] {
                                  startPose.interpolate(endPose, timer.get() / duration)
                                });
                          })
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> {
                            Logger.recordOutput("NoteVisualizer", new Pose3d[] {});
                          });
                },
                Set.of())
            .ignoringDisable(true));
  }

  public static Command ampShoot() {
    return new ScheduleCommand(
        Commands.defer(
                () -> {
                  final Pose3d startPose =
                      new Pose3d(robotPoseSupplier.get()).transformBy(launcherTransform);
                  final boolean isRed =
                      DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Red);
                  final Pose3d endPose =
                      new Pose3d(
                          isRed ? FieldConstants.redAmp : FieldConstants.blueAmp,
                          startPose.getRotation());

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / shotSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  hasNote = false;
                  return Commands.run(
                          () -> {
                            Logger.recordOutput(
                                "NoteVisualizer",
                                new Pose3d[] {
                                  startPose.interpolate(endPose, timer.get() / duration)
                                });
                          })
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> {
                            Logger.recordOutput("NoteVisualizer", new Pose3d[] {});
                          });
                },
                Set.of())
            .ignoringDisable(true));
  }

  /*Position of the note on the robot */
  private static Pose3d getIndexerPose3d(double angleRads) {
    Transform3d indexerTransform =
        new Transform3d(0.1605, 0.0, 0.262, new Rotation3d(0.0, -angleRads, 0.0))
            .plus(new Transform3d(0.63, 0.0, 0.0, new Rotation3d(0, 1.92, 0)));
    return new Pose3d(robotPoseSupplier.get()).transformBy(indexerTransform);
  }

  public static void enableAccurateNotes(double angleRads, double extention) {
    Pose3d pivot = new Pose3d(-0.2, 0.0, 0.31, new Rotation3d(0.0, -angleRads + 0.226893, 0.0));

    Logger.recordOutput("NoteVisualizer/MoreAccurateNotes", getArmDistalPose(pivot, extention));

    getZ = getArmDistalPose(pivot, extention).getZ();
  }

  public static Pose3d getArmDistalPose(Pose3d armProximalPose, double armExtensionLength) {
    return armProximalPose.transformBy(
        new Transform3d(new Translation3d(armExtensionLength, 0.0, 0.0), new Rotation3d()));
  }

  public static double getZ() {
    return getZ;
  }
}
