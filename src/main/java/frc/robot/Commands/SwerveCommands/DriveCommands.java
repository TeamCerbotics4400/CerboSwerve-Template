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

package frc.robot.Commands.SwerveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Swerve.Drive;
import frc.robot.Subsystems.Vision.Limelight.LimelightNotes.LimelightNotes;
import frc.robot.Subsystems.Vision.Limelight.LimelightNotes.LimelightNotesIOSim;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static double kP = 0.4;
  private static CommandXboxController controller = new CommandXboxController(0);
  private static final LimelightNotes ll = new LimelightNotes(new LimelightNotesIOSim());
  private static PIDController aimController = new PIDController(0.3, 0, 0.01);

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    return Commands.run(
        () -> {
          changePID();
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  controller.b().getAsBoolean()
                      ? (Math.pow(ll.getDistanceFromTarget(), 2)
                              - Math.pow(linearVelocity.getY(), 2))
                          * -kP
                      : linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
          SmartDashboard.putNumber(
              "PID VALUE", RobotContainer.getSwerveSubsystem().getRotation().getDegrees());
        },
        drive);
  }

  private static void changePID() {
    if (Math.abs(ll.getTx()) < 0.5) {
      kP = 0;
    } else {
      kP = 0.3;
    }

    if (aimController.calculate(ll.getTx()) < 0) {
      kP = kP * -1;
    } else {
      kP = Math.abs(kP);
    }

    if (RobotContainer.getSwerveSubsystem().getRotation().getDegrees() > 90
        || RobotContainer.getSwerveSubsystem().getRotation().getDegrees() < -90) {
      kP = kP * -1;
    } else {
      kP = kP * -1;
    }
  }
}
