package frc.robot.Commands.AutoCommands.Paths;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.AutoCommands.AutoCommand;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class TestAuto extends AutoCommand {

  private PathPlannerPath startToFirst;

  {
    try {
      startToFirst = PathPlannerPath.fromPathFile("IntakeAssist1");
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  public TestAuto() {

    addCommands(
        Commands.deadline(
            Commands.sequence(
                new PathPlannerAuto("Starting pose 1"), AutoBuilder.followPath(startToFirst))));
  }

  @Override
  public List<Pose2d> getAllPathPoses() {
    return Stream.of(startToFirst.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getStartingPose() {
    return startToFirst.getStartingDifferentialPose();
  }
}
