package frc.team449.robot2023.subsystems.arm

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.WrappedMotor
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log

class Arm(
  private val pivotMotor: WrappedMotor,
  private val jointMotor: WrappedMotor,
  private val feedForward: TwoJointArmFeedForward,
  pivotToJoint: Double,
  jointToEndEffector: Double
) : Loggable, SubsystemBase() {

  private val kinematics = ArmKinematics(
    pivotToJoint,
    jointToEndEffector
  )

  private var desiredState = ArmState(
    Rotation2d(pivotMotor.position),
    Rotation2d(jointMotor.position)
  )

  @get:Log.ToString
  var state: ArmState
    get() = ArmState(
      Rotation2d(pivotMotor.position),
      Rotation2d(jointMotor.position),
      pivotMotor.velocity,
      jointMotor.velocity
    )
    set(state) {
      desiredState = state
    }

  @get:Log.ToString
  val coordinate: CartesianArmState
    get() = kinematics.toCartesian(state)

  override fun periodic() {
    /** TODO PID */
    val u = feedForward.calculate(desiredState.matrix)

    pivotMotor.setVoltage(u[0, 0])
    jointMotor.setVoltage(u[1, 0])
  }
}
