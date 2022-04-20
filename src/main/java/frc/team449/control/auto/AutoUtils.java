package frc.team449.control.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Map;
import java.util.function.Function;

public class AutoUtils {
  /**
   * Insert pauses in an auto drive command while it executes other commands
   *
   * @param fullTraj The complete trajectory
   * @param driveCommandMaker Used for creating auto driving commands from parts of the trajectory
   * @param pauses A map giving times to pause the main trajectory at to execute another Command
   * @return
   */
  public static Command interspersePauses(
      Trajectory fullTraj,
      Function<Trajectory, Command> driveCommandMaker,
      Map<Double, Command> pauses) {
    // todo clean this up
    var pauseTimes = new ArrayList<>(pauses.keySet());
    // Sort in descending order
    pauseTimes.sort(Comparator.reverseOrder());

    Command cmd = null;
    // The list of states for the current stretch of the trajectory
    var currStates = new ArrayList<Trajectory.State>();

    for (var state : fullTraj.getStates()) {
      var lastPauseTime = pauseTimes.get(pauseTimes.size() - 1);
      if (state.timeSeconds > lastPauseTime) {
        var newCmd =
            driveCommandMaker.apply(new Trajectory(currStates)).andThen(pauses.get(lastPauseTime));
        if (cmd == null) {
          cmd = newCmd;
        } else {
          cmd = cmd.andThen(newCmd);
        }
        pauseTimes.remove(pauseTimes.size() - 1);
        currStates = new ArrayList<>();
      }
      currStates.add(state);
    }

    return cmd;
  }
}
