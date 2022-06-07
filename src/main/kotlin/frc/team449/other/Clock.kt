package frc.team449.other

import edu.wpi.first.wpilibj.Timer
import org.jetbrains.annotations.Contract

/**
 * A wrapper on [System].currentTimeMillis that caches the time, to avoid calling the
 * currentTimeMillis method.
 */
object Clock {
  /** The starting time for this clock.  */
  private var startTime = 0.0

  /** The time since startTime, in milliseconds.  */
  private var currentTime = 0.0

  /** Updates the current time.  */
  @Synchronized
  fun updateTime() {
    currentTime = Timer.getFPGATimestamp() - startTime
  }

  /** Sets the start time to the current time.  */
  @Synchronized
  fun setStartTime() {
    startTime = currentTimeSeconds()
  }

  /** The time since the start time, in milliseconds.  */
  @Contract(pure = true)
  @Synchronized
  fun currentTimeMillis(): Long {
    return (currentTime * 1000).toLong()
  }

  /** The time since the start time, in seconds.  */
  @Contract(pure = true)
  @Synchronized
  fun currentTimeSeconds(): Double {
    return currentTime
  }
}
