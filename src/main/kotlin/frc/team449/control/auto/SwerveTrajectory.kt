package frc.team449.control.auto

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d

class SwerveTrajectory(states: Array<DoubleArray>) {
  private val trajectory: MutableList<State>

  init {
    trajectory = ArrayList()
    for (state in states) {
      trajectory.add(
        State(
          state[0],
          Pose2d(Translation2d(state[1], state[2]), Rotation2d(state[3])),
          Vector3d(state[4], state[5], state[6])
        )
      )
    }
  }

  fun getInitialPose(): Pose2d {
    return trajectory[0].pose
  }

  fun getTotalTime(): Double {
    return trajectory[trajectory.size - 1].totalTime
  }

  fun sample(time: Double): State {
    if (time < trajectory[0].totalTime) {
      return trajectory[0]
    }
    if (time > getTotalTime()) {
      return trajectory[trajectory.size - 1]
    }
    var low = 1
    var high = trajectory.size - 1
    while (low != high) {
      val mid = (low + high) / 2
      if (trajectory[mid].totalTime < time) {
        low = mid + 1
      } else {
        high = mid
      }
    }
    val previousState = trajectory[low - 1]
    val currentState = trajectory[low]
    return if (currentState.totalTime - previousState.totalTime == 0.0) {
      State(currentState.totalTime, currentState.pose, currentState.velocity)
    } else previousState.interpolate(currentState, (time - previousState.totalTime) / (currentState.totalTime - previousState.totalTime))
  }

  class State(var totalTime: Double, var pose: Pose2d, var velocity: Vector3d) {

    fun interpolate(end: State, x: Double): State {
      val newT = (end.totalTime - totalTime) * x + totalTime
      val startPoseVector = Vector3d.fromPose(pose)
      val endPoseVector = Vector3d.fromPose(end.pose)
      val newPose = endPoseVector.minus(startPoseVector).times(x).plus(startPoseVector).toPose()
      val newVelocity = end.velocity.minus(velocity).times(x).plus(velocity)
      return State(
        newT,
        newPose,
        newVelocity
      )
    }
  }
}
