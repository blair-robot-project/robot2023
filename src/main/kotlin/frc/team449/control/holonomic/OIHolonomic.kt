package frc.team449.control.holonomic

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Timer
import frc.team449.control.OI
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log
import java.util.function.DoubleSupplier
import kotlin.math.hypot

/**
 * Create an OI for controlling a holonomic drivetrain (probably swerve).
 * The x and y axes on one joystick are used to control x and y velocity (m/s),
 * while the x axis on another joystick is used to control rotational velocity (m/s).
 * <p> The magnitude of the acceleration is clamped
 * <p>Note that the joystick's X
 * axis corresponds to the robot's/field's Y and vice versa
 *
 * @param drive The drivetrain this OI is controlling
 * @param xThrottle The Y axis of the strafing joystick
 * @param yThrottle The X axis of the strafing joystick
 * @param rotThrottle The X axis of the rotating joystick
 * @param rotRamp Used to ramp angular velocity
 * @param maxAccel Max accel, used for ramping
 * @param fieldOriented Whether the OI x and y translation should
 * be relative to the field rather than relative to the robot. This better be true.
 */
class OIHolonomic(
  @Log.Exclude
  val drive: HolonomicDrive,
  private val xThrottle: DoubleSupplier,
  private val yThrottle: DoubleSupplier,
  private val rotThrottle: DoubleSupplier,
  private val rotRamp: SlewRateLimiter,
  private val maxAccel: Double,
  private val fieldOriented: Boolean
) : OI, Loggable, Sendable {

  /** Previous x velocity (scaled and clamped) */
  @Log
  private var prevX = 0.0

  /** Previous y velocity (scaled and clamped) */
  @Log
  private var prevY = 0.0

  private var prevTime = Double.NaN

  // todo Remove these soon, only for logging
  private var dx = 0.0
  private var dy = 0.0
  private var magAcc = 0.0
  private var dt = 0.0
  private var magAccClamped = 0.0

  /**
   * @return The [ChassisSpeeds] for the given x, y and
   * rotation input from the joystick */
  override fun get(): ChassisSpeeds {
    var currTime = Timer.getFPGATimestamp()
    if (this.prevTime.isNaN()) {
      this.prevTime = currTime - 0.02
    }
    this.dt = currTime - prevTime
    this.prevTime = currTime

    var xScaled = xThrottle.asDouble * drive.maxLinearSpeed
    var yScaled = yThrottle.asDouble * drive.maxLinearSpeed

    // Clamp the acceleration
    this.dx = xScaled - this.prevX
    this.dy = yScaled - this.prevY
    this.magAcc = hypot(dx / dt, dy / dt)
    this.magAccClamped = MathUtil.clamp(magAcc, -this.maxAccel, this.maxAccel)

    // Scale the change in x and y the same as the acceleration
    var factor = if (magAcc == 0.0) 0.0 else magAccClamped / magAcc
    var dxClamped = dx * factor
    var dyClamped = dy * factor
    var xClamped = prevX + dxClamped
    var yClamped = prevY + dyClamped

    this.prevX = xClamped
    this.prevY = yClamped

    var rotRaw = rotThrottle.asDouble
    var rotScaled = rotRamp.calculate(rotRaw * drive.maxRotSpeed)

    return if (this.fieldOriented) {
      ChassisSpeeds.fromFieldRelativeSpeeds(
        xClamped,
        yClamped,
        rotScaled,
        drive.heading
      )
    } else {
      ChassisSpeeds(xClamped, yClamped, rotScaled)
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.addDoubleProperty("currX", this.xThrottle::getAsDouble, null)
    builder.addDoubleProperty("currY", this.yThrottle::getAsDouble, null)
    builder.addDoubleProperty("prevX", { this.prevX }, null)
    builder.addDoubleProperty("prevY", { this.prevY }, null)
    builder.addDoubleProperty("dx", { this.dx }, null)
    builder.addDoubleProperty("dy", { this.dy }, null)
    builder.addDoubleProperty("dt", { this.dt }, null)
    builder.addDoubleProperty("magAcc", { this.magAcc }, null)
    builder.addDoubleProperty("magAccClamped", { this.magAccClamped }, null)
    builder.addStringProperty("speeds", { this.get().toString() }, null)
  }
}
