/*package frc.robot.Commands.AutoCommands.Paths;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.AutoCommands.AutoCommand;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class FiveNoteAutoPath extends AutoCommand {

  private final PathPlannerPath startToFirst;
  private final PathPlannerPath secondToFirst;
  private final PathPlannerPath thirdToFirst;
  private final PathPlannerPath fourthoFirst;
  private final PathPlannerPath fifthToFirst;
  private final PathPlannerPath sixToFirst;
  private final PathPlannerPath sevenToFirst;

  public FiveNoteAutoPath() {
    startToFirst = PathPlannerPath.fromPathFile("5NoteAuto1");
    secondToFirst = PathPlannerPath.fromPathFile("5NoteAuto2");
    thirdToFirst = PathPlannerPath.fromPathFile("5NoteAuto3");
    fourthoFirst = PathPlannerPath.fromPathFile("5NoteAuto4");
    fifthToFirst = PathPlannerPath.fromPathFile("5NoteAuto5");
    sixToFirst = PathPlannerPath.fromPathFile("5NoteAuto6");
    sevenToFirst = PathPlannerPath.fromPathFile("5NoteAuto7");

    addCommands(Commands.deadline(Commands.sequence(new PathPlannerAuto("6NoteAuto"))));
  }

  @Override
  public List<Pose2d> getAllPathPoses() {
    return Stream.of(
            startToFirst.getPathPoses(),
            secondToFirst.getPathPoses(),
            thirdToFirst.getPathPoses(),
            fourthoFirst.getPathPoses(),
            fifthToFirst.getPathPoses(),
            sixToFirst.getPathPoses(),
            sevenToFirst.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getStartingPose() {
    return startToFirst
        .getTrajectory(new ChassisSpeeds(), new Rotation2d())
        .getInitialTargetHolonomicPose();
  }
}
*/
