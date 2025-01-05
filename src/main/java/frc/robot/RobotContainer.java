// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Util.LocalADStarAK;
import frc.Util.Logging.LoggedDashboardChooser;
import frc.Util.NoteVisualizer;
import frc.robot.Commands.AutoCommands.AutoCommand;
import frc.robot.Commands.AutoCommands.Paths.NoneAuto;
import frc.robot.Commands.AutoCommands.Paths.TestAuto;
import frc.robot.Commands.SwerveCommands.DriveCommands;
import frc.robot.Commands.SwerveCommands.NoteAlignCommand;
import frc.robot.Subsystems.Swerve.Drive;
import frc.robot.Subsystems.Swerve.GyroIO;
import frc.robot.Subsystems.Swerve.GyroIOPigeon2;
import frc.robot.Subsystems.Swerve.ModuleIO;
import frc.robot.Subsystems.Swerve.ModuleIOSim;
import frc.robot.Subsystems.Swerve.ModuleIOTalonFX;
import frc.robot.Subsystems.Vision.Limelight.LimelightNotes.LimelightNotes;
import frc.robot.Subsystems.Vision.Limelight.LimelightNotes.LimelightNotesIOSim;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  // nopublic static SimDefenseBot defenseBot = new SimDefenseBot(2);

  private static final CommandXboxController chassisDriver = new CommandXboxController(0);
  private static final CommandXboxController subsystemsDriver = new CommandXboxController(1);

  private static LoggedDashboardChooser<AutoCommand> autoChooser;

  public static Field2d autoPreviewField = new Field2d();

  public static Drive drive;

  public static LimelightNotes llNotes = new LimelightNotes(new LimelightNotesIOSim());

  /*public static PhotonSim frontLeftCamera;
  public static PhotonSim frontRightCamera;
  public static PhotonSim backLeftCamera;
  public static PhotonSim backRightCamera;*/

  // public static PhotonAprilTagVision aprilTagVision;

  public RobotContainer() {
    /** Options for the current mode of the robot */
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));    

        /*aprilTagVision =
        new PhotonAprilTagVision(
            drive, frontLeftCamera, frontRightCamera, backLeftCamera, backRightCamera);*/
        break;
        // --------------------------------------------
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
    
        /*frontLeftCamera = new PhotonSim(0);
        frontRightCamera = new PhotonSim(1);
        backLeftCamera = new PhotonSim(2);
        backRightCamera = new PhotonSim(3);

        aprilTagVision =
            new PhotonAprilTagVision(
                drive, frontLeftCamera, frontRightCamera, backLeftCamera, backRightCamera);*/
        break;
        // --------------------------------------------
      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        
        /*frontLeftCamera = new PhotonSim(0);
        frontRightCamera = new PhotonSim(2);
        backLeftCamera = new PhotonSim(3);
        backRightCamera = new PhotonSim(1);

        aprilTagVision = new PhotonAprilTagVision(drive, frontLeftCamera);*/

        break;
        // --------------------------------------------
    }

    registerNamedCommands();
    /** Visualisation of the current auto selected * */
    autoChooser = new LoggedDashboardChooser<>("Auto Mode");

    autoChooser.onChange(
        auto -> {
          autoPreviewField.getObject("path").setPoses(auto.getAllPathPoses());
        });

    /** Auto options */
    autoChooser.addDefaultOption("None", new NoneAuto());
    // autoChooser.addOption("Complement auto", new ComplementPath());
    // autoChooser.addOption("Six Note Auto", new FiveNoteAutoPath());
    // autoChooser.addOption("Test Of Change", new ChangeTest(drive));
    autoChooser.addOption("Test", new TestAuto());

    PathPlannerLogging.setLogActivePathCallback(
        (poses -> Logger.recordOutput("Swerve/ActivePath", poses.toArray(new Pose2d[0]))));
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Swerve/TargetPathPose", pose));

    SmartDashboard.putData("Auto Preview", autoPreviewField);

    SmartDashboard.putString("Current Robot mode", Constants.currentMode.toString());

    Pathfinding.setPathfinder(new LocalADStarAK());

    // Set up note visualizer
    NoteVisualizer.setRobotPoseSupplier(drive::getPose);

    configureBindings();
  }

  private void configureBindings() {

    /* Control 1 commands */
    // Chassis commands
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -chassisDriver.getLeftY(),
            () -> -chassisDriver.getLeftX(),
            () -> -chassisDriver.getRightX()));

    // Lock modules
    chassisDriver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Set field centric
    chassisDriver.a().onTrue(drive.runOnce(() -> drive.zeroHeading()));

    // AutoRoutines
    /*chassisDriver
    .povUp()
    .toggleOnTrue(
        Commands.sequence(
            pathfindAndAlignAmp().until(chassisDriver.axisGreaterThan(1, 0.1)),
            /*(() ->
            drive
                    .getPose()
                    .getTranslation()
                    .getDistance(
                        Robot.isRedAlliance()
                            ? FieldConstants.redAmpPose.getTranslation()
                            : FieldConstants.blueAmpPose.getTranslation())
                <= 1.5),*/
    //   m_arm.goToPosition(12, m_arm.changeState(ArmStates.STANDING))));

    // chassisDriver.povDown().toggleOnTrue(pathfindAndAlignAmp());

    // chassisDriver.povLeft().toggleOnTrue(pathfindAndAlignSource());

    // Control rumbles when game piece is detected
    chassisDriver
        .rightBumper()
        .and(() -> NoteVisualizer.hasSimNote())
        .onTrue(controllerRumbleCommand().withTimeout(1));

    chassisDriver.leftBumper().whileTrue(new NoteAlignCommand(drive));

    chassisDriver.y().onTrue(new InstantCommand(() -> drive.changeIntakeAssist()));
  }

  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          chassisDriver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          chassisDriver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  /*public static Command pathfindAndAlignAmp() {
    return Commands.either(
        drive
            .goToPose(FieldConstants.redAmpPose)
            .until(() -> Math.abs(chassisDriver.getRawAxis(1)) > 0.1),
        drive
            .goToPose(FieldConstants.blueAmpPose)
            .until(() -> Math.abs(chassisDriver.getRawAxis(1)) > 0.1),
        Robot::isRedAlliance);
  }

  public static Command pathfindAndAlignSource() {
    return Commands.either(
        drive
            .goToPose(FieldConstants.redPickupPose)
            .until(() -> Math.abs(chassisDriver.getRawAxis(1)) > 0.1),
        drive
            .goToPose(FieldConstants.bluePickupPose)
            .until(() -> Math.abs(chassisDriver.getRawAxis(1)) > 0.1),
        Robot::isRedAlliance);
  }*/

  public void registerNamedCommands() {
    //NamedCommands.registerCommand("ShootSim", NoteVisualizer.speakerShoot());
    NamedCommands.registerCommand(
        "ActivateIntakeAssist", new InstantCommand(() -> drive.changeIntakeAssist()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public static Drive getSwerveSubsystem() {
    return drive;
  }
}
