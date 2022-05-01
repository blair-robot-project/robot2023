package frc.team449.control.auto

import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj2.command.Command
import java.util.function.Function

object AutoUtils {
  /**
   * Insert pauses in an auto drive command while it executes other commands
   *
   * @param fullTraj The complete trajectory
   * @param driveCommandMaker Used for creating auto driving commands from parts of the trajectory
   * @param pauses A map giving times to pause the main trajectory at to execute another Command
   * @return
   */
  fun interspersePauses(
    fullTraj: Trajectory,
    driveCommandMaker: Function<Trajectory?, Command>,
    pauses: Map<Double, Command?>
  ): Command? {
    // todo clean this up
    val pauseTimes = pauses.keys.toMutableList()
    // Sort in descending order
    pauseTimes.sortWith(Comparator.reverseOrder())
    var cmd: Command? = null
    // The list of states for the current stretch of the trajectory
    var currStates = ArrayList<Trajectory.State?>()
    for (state in fullTraj.states) {
      val lastPauseTime = pauseTimes[pauseTimes.size - 1]
      if (state.timeSeconds > lastPauseTime) {
        val newCmd = driveCommandMaker.apply(Trajectory(currStates)).andThen(pauses[lastPauseTime])
        cmd = if (cmd == null) {
          newCmd
        } else {
          cmd.andThen(newCmd)
        }
        pauseTimes.removeAt(pauseTimes.size - 1)
        currStates = ArrayList()
      }
      currStates.add(state)
    }
    return cmd
  }
}
