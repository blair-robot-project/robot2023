package frc.team449.control.holonomic

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase
import frc.team449.system.motor.WrappedMotor
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log

open class SwerveModule constructor(
  private val name: String,
  private val drivingMotor: WrappedMotor,
  private val turningMotor: WrappedMotor,
  private val driveController: PIDController,
  private val turnController: ProfiledPIDController,
  private val driveFeedforward: SimpleMotorFeedforward,
  private val turnFeedforward: SimpleMotorFeedforward,
  @Log.ToString val location: Translation2d
) : Loggable {
  init {
    turnController.enableContinuousInput(-Math.PI, Math.PI)
  }

  open var state: SwerveModuleState
    get() {
      return SwerveModuleState(
        drivingMotor.velocity,
        Rotation2d(turningMotor.position)
      )
    }
    set(desiredState) {
      // Ensure the module doesn't turn the long way around
      val state = SwerveModuleState.optimize(
        desiredState,
        Rotation2d(turningMotor.position)
      )

      val drivePid = driveController.calculate(
        drivingMotor.velocity,
        state.speedMetersPerSecond
      )
      val driveFF = driveFeedforward.calculate(state.speedMetersPerSecond)
      drivingMotor.setVoltage(drivePid + driveFF)

      val turnPid = turnController.calculate(
        turningMotor.velocity,
        state.angle.radians
      )
      val turnFF = turnFeedforward.calculate(
        turnController.setpoint.velocity
      )
      turningMotor.setVoltage(turnPid + turnFF)
    }

  override fun configureLogName() = this.name

  companion object {
    fun create(
      name: String,
      drivingMotor: WrappedMotor,
      turningMotor: WrappedMotor,
      driveController: PIDController,
      turnController: ProfiledPIDController,
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
  turnController: ProfiledPIDController,
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
