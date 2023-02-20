package frc.team449.control.holonomic

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import frc.team449.control.OI
import frc.team449.robot2023.constants.RobotConstants
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot

/**
 * Create an OI for controlling a holonomic drivetrain.
 * The x and y axes on one joystick are used to control x and y velocity (m/s),
 * while the x axis on another joystick is used to control rotational velocity (m/s).
 * This OI will also map A, X, B, and Y to 90 degree setpoints.
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
class OrthogonalHolonomicOI(
  private val drive: HolonomicDrive,
  private val xThrottle: DoubleSupplier,
  private val yThrottle: DoubleSupplier,
  private val rotThrottle: DoubleSupplier,
  private val rotRamp: SlewRateLimiter,
  private val maxAccel: Double,
  private val fieldOriented: () -> Boolean,
  private val controller: PIDController,
  private val yButton: BooleanSupplier,
  private val xButton: BooleanSupplier,
  private val aButton: BooleanSupplier,
  private val bButton: BooleanSupplier,
) : OI, Sendable {

  init {
    controller.enableContinuousInput(0.0, 2 * PI)
    controller.setTolerance(0.1)
  }

  /** Previous x velocity (scaled and clamped) */
  private var prevX = 0.0

  /** Previous y velocity (scaled and clamped) */
  private var prevY = 0.0

  private var prevTime = Double.NaN

  private var dx = 0.0
  private var dy = 0.0
  private var magAcc = 0.0
  private var dt = 0.0
  private var magAccClamped = 0.0

  private var rotScaled = 0.0
  private val allianceCompensation = if (RobotConstants.ALLIANCE_COLOR == DriverStation.Alliance.Red) 0.0 else PI

  /**
   * @return The [ChassisSpeeds] for the given x, y and
   * rotation input from the joystick */
  override fun get(): ChassisSpeeds {
    val currTime = Timer.getFPGATimestamp()
    if (this.prevTime.isNaN()) {
      this.prevTime = currTime - 0.02
    }
    this.dt = currTime - prevTime
    this.prevTime = currTime

    val xScaled = xThrottle.asDouble * drive.maxLinearSpeed
    val yScaled = yThrottle.asDouble * drive.maxLinearSpeed

    // Clamp the acceleration
    this.dx = xScaled - this.prevX
    this.dy = yScaled - this.prevY
    this.magAcc = hypot(dx / dt, dy / dt)
    this.magAccClamped = MathUtil.clamp(magAcc, -this.maxAccel, this.maxAccel)

    // Scale the change in x and y the same as the acceleration
    val factor = if (magAcc == 0.0) 0.0 else magAccClamped / magAcc
    val dxClamped = dx * factor
    val dyClamped = dy * factor
    val xClamped = prevX + dxClamped
    val yClamped = prevY + dyClamped

    this.prevX = xClamped
    this.prevY = yClamped

    /** Based on which button was pressed, give in the setpoint to the PID controller.
     *  You must use calculate because the atSetpoint() method requires there to have
     *  been a measurement and setpoint. */
    if (yButton.asBoolean) {
      controller.calculate(
        drive.pose.rotation.radians,
        (0.0 + allianceCompensation) % 2 * PI
      ) * drive.maxRotSpeed
    } else if (xButton.asBoolean) {
      controller.calculate(
        drive.pose.rotation.radians,
        (PI / 2 + allianceCompensation) % 2 * PI
      ) * drive.maxRotSpeed
    } else if (aButton.asBoolean) {
      controller.calculate(
        drive.pose.rotation.radians,
        (PI + allianceCompensation) % 2 * PI
      ) * drive.maxRotSpeed
    } else if (bButton.asBoolean) {
      controller.calculate(
        drive.pose.rotation.radians,
        (3 * PI / 2 + allianceCompensation) % 2 * PI
      ) * drive.maxRotSpeed
    } else {
      controller.calculate(
        0.0,
        0.0
      )
    }

    /** If the PID controller is at its setpoint, then allow the driver to control rotation,
     * otherwise let the PID do its thing. */
    rotScaled = if (controller.atSetpoint()) {
      rotRamp.calculate(rotThrottle.asDouble * drive.maxRotSpeed)
    } else {
      controller.calculate(drive.pose.rotation.radians) * drive.maxRotSpeed
    }

    // translation velocity vector
    val vel = Translation2d(xClamped, yClamped)

    return if (this.fieldOriented()) {
      /** Quick fix for the velocity skewing towards the direction of rotation
       * by rotating it with offset proportional to how much we are rotating
       **/
      vel.rotateBy(Rotation2d(-rotScaled * dt / 2))
      ChassisSpeeds.fromFieldRelativeSpeeds(
        vel.x,
        vel.y,
        rotScaled,
        drive.pose.rotation
      )
    } else {
      ChassisSpeeds(
        vel.x,
        vel.y,
        rotScaled
      )
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

  companion object {
    fun createOrthogonalHolonomicOI(drive: HolonomicDrive, driveController: XboxController): OrthogonalHolonomicOI {
      return OrthogonalHolonomicOI(
        drive,
        { if (abs(driveController.leftY) < RobotConstants.TRANSLATION_DEADBAND) .0 else -driveController.leftY },
        { if (abs(driveController.leftX) < RobotConstants.TRANSLATION_DEADBAND) .0 else -driveController.leftX },
        { if (abs(driveController.getRawAxis(4)) < RobotConstants.ROTATION_DEADBAND) .0 else -driveController.getRawAxis(4) },
        SlewRateLimiter(RobotConstants.RATE_LIMIT),
        RobotConstants.MAX_ACCEL,
        { !driveController.leftBumper },
        RobotConstants.ORTHOGONAL_CONRTOLLER,
        { driveController.yButtonPressed },
        { driveController.xButtonPressed },
        { driveController.aButtonPressed },
        { driveController.bButtonPressed }
      )
    }
  }
}
