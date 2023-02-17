package frc.team449.robot2023.subsystems.outreach.shooter

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.WrappedMotor
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log
import kotlin.math.abs

class Shooter(
  private val shooterMotor: WrappedMotor,
  private val feederMotor: WrappedMotor
) : SubsystemBase(), Loggable {

  private var runShoot = false
  private var shooterController = PIDController(
    ShooterConstants.SHOOTER_KP,
    ShooterConstants.SHOOTER_KI,
    ShooterConstants.SHOOTER_KD
  )
  private var shooterFF = SimpleMotorFeedforward(
    ShooterConstants.SHOOTER_KS,
    ShooterConstants.SHOOTER_KV,
    ShooterConstants.SHOOTER_KA
  )

  @Log.Graph
  private var shooterSpeed = 0.0

  // Starts the shooter by changing runShoot to true.
  fun runShooter() {
    runShoot = true
  }

  // Stops the shooter by changing runShoot to false.
  fun stopShooter() {
    runShoot = false
  }

  fun runShooterReverse() {
    shooterMotor.setVoltage(-7.0)
  }

  private fun shooterAtSpeed(desiredSpeed: Double): Boolean {
    if (abs(shooterMotor.encoder.velocity - desiredSpeed) <= ShooterConstants.TOLERANCE) {
      return true
    }
    return false
  }

  // Uses PID and FF to calculate motor voltage.
  override fun periodic() {
    shooterSpeed = shooterMotor.velocity
    if (runShoot) {

      val shooterPID = shooterController.calculate(shooterMotor.velocity, ShooterConstants.SHOOTER_VEL)
      val shooterFF = shooterFF.calculate(ShooterConstants.SHOOTER_VEL)

      shooterMotor.setVoltage(shooterPID + shooterFF)
      if (shooterAtSpeed(ShooterConstants.SHOOTER_VEL)) {
        feederMotor.setVoltage(ShooterConstants.FEEDER_VOLTAGE)
      }
    } else {
      shooterMotor.set(0.0)
      feederMotor.set(0.0)
    }
  }
}
