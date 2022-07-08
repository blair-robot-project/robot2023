package frc.team449.control.auto

import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import java.util.*
import java.util.function.Function
import kotlin.Comparator
import kotlin.collections.ArrayList

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
  /**
   * @param timeStamp List of commands paired with desired time after beginning auto they are to be scheduled
   *
   */
  fun autoSequence(
    timeStamp: List<Pair<Double, Command>> // need a list of Time mapped to Commands to do in auto
  ): SequentialCommandGroup {
    val queue = PriorityQueue<Pair<Double, Command>>(
      timeStamp.size
    ) { o1, o2 ->
      val a = o1.first
      val b = o2.first
      if (a >= b) if (a == b) 0 else -1
      1
    }
    timeStamp.forEach { queue.offer(it) }
    val cmd = SequentialCommandGroup()
    var lastTime = 0.0
    while (!queue.isEmpty()) {
      val current = queue.poll()
      println(current.first)
      cmd.addCommands(
        // wait command
        WaitCommand(current.first - lastTime),
        // actual command
        current.second
      )
      lastTime = current.first
    }
    if (lastTime < 15.0) cmd.addCommands(WaitCommand(15.0 - lastTime))
    return cmd
  }
}
