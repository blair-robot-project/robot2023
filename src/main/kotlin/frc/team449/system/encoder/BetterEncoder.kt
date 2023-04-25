package frc.team449.system.encoder

import edu.wpi.first.hal.SimDevice
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.*

class BetterEncoder(
  val name: String,
  val dutyCycle: DutyCycle,
  var offset: Double,
  private val microsecondsCycleTime: Double,
  val inverted: Boolean,
  private val sensorMin: Double,
  private val sensorMax: Double
) : Sendable {

  private val simDevice = SimDevice.create("DutyCycle:DutyCycleEncoder", dutyCycle.sourceChannel)
  private val simPosition = simDevice?.createDouble("position", SimDevice.Direction.kInput, 0.0)
  private val simDistancePerRotation = simDevice?.createDouble("distance_per_rot", SimDevice.Direction.kOutput, 1.0)
  private val simAbsolutePosition = simDevice?.createDouble("absPosition", SimDevice.Direction.kInput, 0.0)
  private val simIsConnected = simDevice?.createBoolean("connected", SimDevice.Direction.kInput, true)

  private val counter = if (simDevice == null) Counter() else null
  private val analogTrigger = if (simDevice == null) AnalogTrigger(dutyCycle) else null

  private var prevPos = Double.NaN
  private var prevTime = Double.NaN

  init {
    analogTrigger?.setLimitsDutyCycle(0.25, 0.75)
    counter?.setUpSource(analogTrigger, AnalogTriggerOutput.AnalogTriggerType.kRisingPulse)
    counter?.setDownSource(analogTrigger, AnalogTriggerOutput.AnalogTriggerType.kFallingPulse)

    SendableRegistry.addLW(this, "DutyCycle Encoder", dutyCycle.sourceChannel)
  }

  /**
   * Returns the position of the motor from the specified bounds
   */
  fun getPosition(): Double {
    val microsecondsHigh = dutyCycle.highTimeNanoseconds / 1000
    val microsecondsTotal = microsecondsCycleTime
    var position = microsecondsHigh / microsecondsTotal

    // Maps the position onto your desired ranges
    if (position < sensorMin) {
      position = sensorMin
    } else if (position > sensorMax) {
      position = sensorMax
    } else {
      position = (position - sensorMin) / (sensorMax - sensorMin)
    }

    // Accounts for offset
    position = if (inverted) {
      1 - (position - offset) % 1
    } else {
      position - offset % 1
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
}
