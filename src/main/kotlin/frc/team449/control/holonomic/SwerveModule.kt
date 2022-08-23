package frc.team449.control.holonomic

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase
import frc.team449.system.motor.WrappedMotor
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log
import kotlin.math.PI
import kotlin.math.abs

open class SwerveModule constructor(
  private val name: String,
  private val drivingMotor: WrappedMotor,
  private val turningMotor: WrappedMotor,
  private val driveController: PIDController,
  private val turnController: PIDController,
  private val driveFeedforward: SimpleMotorFeedforward,
  private val turnFeedforward: SimpleMotorFeedforward, // Todo remove?
  val location: Translation2d
) : Loggable {
  init {
    turnController.enableContinuousInput(-PI, PI)
    turnController.setTolerance(.1) // Tolerate the noise from the encoders, ~.08 - .09
    driveController.reset()
    turnController.reset()
  }

  private var desiredSpeed = 0.0
  private var desiredAngle = turningMotor.position

  open var state: SwerveModuleState
    @Log.ToString
    get() {
      return SwerveModuleState(
        drivingMotor.velocity,
        Rotation2d(turningMotor.position - PI)
      )
    }
    set(desiredState) {
      // Ensure the module doesn't turn the long way around
      if (abs(desiredState.speedMetersPerSecond) < .001) {
        stop()
        return
      }
      val state = SwerveModuleState.optimize(
        desiredState,
        Rotation2d(turningMotor.position - PI)
      )
      desiredAngle = state.angle.radians
      desiredSpeed = state.speedMetersPerSecond
    }

  fun stop() {
    desiredAngle = turningMotor.position - PI
    desiredSpeed = 0.0
  }
  override fun configureLogName() = this.name

  fun update() {
    val drivePid = driveController.calculate(
      drivingMotor.velocity,
      desiredSpeed
    )
    val driveFF = driveFeedforward.calculate(desiredSpeed)
    drivingMotor.setVoltage(drivePid + driveFF)

    val turnPid = turnController.calculate(
      turningMotor.position - PI,
      desiredAngle
    )

    turningMotor.set(turnPid)
  }

  companion object {
    fun create(
      name: String,
      drivingMotor: WrappedMotor,
      turningMotor: WrappedMotor,
      driveController: PIDController,
      turnController: PIDController,
      driveFeedforward: SimpleMotorFeedforward,
      turnFeedforward: SimpleMotorFeedforward,
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
          turnFeedforward,
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
          turnFeedforward,
          location
        )
      }
    }
  }
}

/**
 * A "simulated" swerve module that just pretends it immediately got to whatever desired state was
 * given
 */
class SwerveModuleSim(
  name: String,
  drivingMotor: WrappedMotor,
  turningMotor: WrappedMotor,
  driveController: PIDController,
  turnController: PIDController,
  driveFeedforward: SimpleMotorFeedforward,
  turnFeedforward: SimpleMotorFeedforward,
  location: Translation2d
) : SwerveModule(
  name,
  drivingMotor,
  turningMotor,
  driveController,
  turnController,
  driveFeedforward,
  turnFeedforward,
  location
) {

  override var state = SwerveModuleState()
}
