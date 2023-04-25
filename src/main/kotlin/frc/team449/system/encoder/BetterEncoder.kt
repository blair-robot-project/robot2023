package frc.team449.system.encoder

import edu.wpi.first.hal.SimDevice
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.*
import frc.team449.robot2023.constants.RobotConstants

class BetterEncoder(
  val name: String,
  val dutyCycle: DutyCycle,
  var offset: Double,
  private val microsecondsCycleTime: Double,
  val inverted: Boolean,
  private val sensorMin: Double,
  private val sensorMax: Double,
  private val unitMin: Double,
  private val unitMax: Double
) : Sendable {

  private val simDevice = SimDevice.create("DutyCycle:DutyCycleEncoder", dutyCycle.sourceChannel)
  private val simPosition = simDevice?.createDouble("position", SimDevice.Direction.kInput, 0.0)
  private val simDistancePerRotation = simDevice?.createDouble("distance_per_rot", SimDevice.Direction.kOutput, 1.0)
  private val simAbsolutePosition = simDevice?.createDouble("absPosition", SimDevice.Direction.kInput, 0.0)
  private val simIsConnected = simDevice?.createBoolean("connected", SimDevice.Direction.kInput, true)

  private var prevPos = Double.NaN
  private var prevTime = Double.NaN

  init {
    SendableRegistry.addLW(this, "DutyCycle Encoder", dutyCycle.sourceChannel)
  }

  /**
   * Returns the position of the motor from the specified bounds
   */
  fun getPosition(): Double {
    val microsecondsHigh = dutyCycle.highTimeNanoseconds / 1000
    val microsecondsTotal = microsecondsCycleTime
    var position = microsecondsHigh / microsecondsTotal

    // Maps the output from 0 to 1
    if (position < sensorMin) {
      position = sensorMin
    } else if (position > sensorMax) {
      position = sensorMax
    } else {
      position = (position - sensorMin) / (sensorMax - sensorMin)
    }

    // Scales the units
    position = position * (unitMax - unitMin) + unitMin

    // Accounts for offset
    val range = unitMax - unitMin
    position = if (inverted) {
      range - (position - offset) % range
    } else {
      (position - offset) % range
    }

    return position
  }

  /**
   * Resets the current encoder position to 0
   */
  fun resetPosition(pos: Double) {
    offset += getPosition() - pos
  }

  /**
   * Gets the angular velocity of the motor
   */
  fun getAngularVelocity(): Double {
    val currentPos = getPosition()
    val currentTime = Timer.getFPGATimestamp()

    val velocity = if (prevPos.isNaN() || prevTime.isNaN()) {
      0.0
    } else {
      val dx = (currentPos - prevPos) % 1
      val dt = currentTime - prevTime
      dx / dt
    }

    return velocity
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.setSmartDashboardType("Encoder")
  }

  companion object {
    fun create(
      name: String,
      channel: Int,
      offset: Double,
      inverted: Boolean,
      unitMin: Double,
      unitMax: Double
    ): BetterEncoder {
      return BetterEncoder(
        name,
        DutyCycle(DigitalInput(channel)),
        offset,
        RobotConstants.CYCLE_TIME,
        inverted,
        RobotConstants.SENSOR_MIN,
        RobotConstants.SENSOR_MAX,
        unitMin,
        unitMax
      )
    }
  }
}
