package frc.team449.control.holonomic

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import frc.team449.system.motor.WrappedMotor
import io.github.oblarg.oblog.Loggable
import kotlin.math.PI
import kotlin.math.abs

/**
 * @param name the name of the module (relevant for logging)
 * @param drivingMotor the motor that controls the speed of the module
 * @param turningMotor the motor that controls the turning(angle) of the module
 * @param driveController the velocity control PID for speed of the module
 * @param turnController the position control PID for turning(angle) of the module
 * @param driveFeedforward voltage predicting equation for a specified speed of the module
 * @param location the location of the module in reference to the center of the robot
 * NOTE: In relation to the robot [+X is forward, +Y is left, and +THETA is Counter Clock-Wise].
 */
open class SwerveModule constructor(
  private val name: String,
  private val drivingMotor: WrappedMotor,
  private val turningMotor: WrappedMotor,
  private val driveController: PIDController,
  private val turnController: PIDController,
  private val driveFeedforward: SimpleMotorFeedforward,
  val location: Translation2d
) : Loggable {
  init {
    turnController.enableContinuousInput(.0, 2 * PI)
    driveController.reset()
    turnController.reset()
  }

  private var desiredSpeed = 0.0
  private var prevDesiredSpeed = 0.0
  private var prevTime = Double.NaN

  open var state: SwerveModuleState
    get() {
      return SwerveModuleState(
        drivingMotor.velocity,
        Rotation2d(turningMotor.position)
      )
    }
    set(desiredState) {
      if (abs(desiredState.speedMetersPerSecond) < .001) {
        stop()
        return
      }
      /** Ensure the module doesn't turn the long way around (more than 90 deg) */
      val state = SwerveModuleState.optimize(
        desiredState,
        Rotation2d(turningMotor.position)
      )
      turnController.setpoint = state.angle.radians
      prevDesiredSpeed = desiredSpeed
      desiredSpeed = state.speedMetersPerSecond
      driveController.setpoint = state.speedMetersPerSecond
    }

  /** Keep same direction of module but keep speed at zero */
  fun stop() {
    turnController.setpoint = turningMotor.position
    desiredSpeed = 0.0
  }
  override fun configureLogName() = this.name

  fun update() {
    /** calculate difference in time from last updated time
     *  to be used for obtaining what acceleration should
     *  be fed to the module
     **/
    val currTime = Timer.getFPGATimestamp()
    if (prevTime.isNaN())
      prevTime = currTime - 0.02
    val dt = currTime - prevTime

    /** CONTROL speed of module */
    val drivePid = driveController.calculate(
      drivingMotor.velocity
    )
    val driveFF = driveFeedforward.calculate(
      prevDesiredSpeed,
      desiredSpeed,
      dt
    )
    drivingMotor.setVoltage(drivePid + driveFF)

    /** CONTROL direction of module */
    val turnPid = turnController.calculate(
      turningMotor.position
    )
    turningMotor.set(turnPid)

    prevTime = currTime
  }

  /**
   * Creates a simulated or a real robot based
   * on if the robot is being simulated.
   * @see SwerveModule for parameter description
   */
  companion object {
    fun create(
      name: String,
      drivingMotor: WrappedMotor,
      turningMotor: WrappedMotor,
      driveController: PIDController,
      turnController: PIDController,
      driveFeedforward: SimpleMotorFeedforward,
      location: Translation2d
    ): SwerveModule {
      if (RobotBase.isReal()) {
        return SwerveModule(
          name,
          drivingMotor,
          turningMotor,
          driveController,
          turnController,
          driveFeedforward,
          location
        )
      } else {
        return SwerveModuleSim(
          name,
          drivingMotor,
          turningMotor,
          driveController,
          turnController,
          driveFeedforward,
          location
        )
      }
    }
  }
}

/**
 * A "simulated" swerve module that just pretends
 * it immediately got to whatever desired state was given
 */
class SwerveModuleSim(
  name: String,
  drivingMotor: WrappedMotor,
  turningMotor: WrappedMotor,
  driveController: PIDController,
  turnController: PIDController,
  driveFeedforward: SimpleMotorFeedforward,
  location: Translation2d
) : SwerveModule(
  name,
  drivingMotor,
  turningMotor,
  driveController,
  turnController,
  driveFeedforward,
  location
) {

  override var state = SwerveModuleState()
}
