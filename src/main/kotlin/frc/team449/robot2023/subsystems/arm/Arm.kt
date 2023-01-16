package frc.team449.robot2023.subsystems.arm

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.WrappedMotor

class Arm(
  private val baseMotor: WrappedMotor,
  private val pivotMotor: WrappedMotor,
  pivotToJoint: Double,
  jointToEndEffector: Double
) : SubsystemBase() {

  private val kinematics = ArmKinematics(
    pivotToJoint,
    jointToEndEffector
  )

  private var desiredState = ArmState(
    Rotation2d(baseMotor.position),
    Rotation2d(pivotMotor.position)
  )

  var state: ArmState
    get() = ArmState(
      Rotation2d(baseMotor.position),
      Rotation2d(pivotMotor.position),
      baseMotor.velocity,
      pivotMotor.velocity
    )
    set(state) {
      desiredState = state
    }

  val coordinate: CartesianArmState
    get() = kinematics.toCartesian(state)
  override fun periodic() {
    /** TODO control theory application setVoltage(FF + PID) */
  }
}
