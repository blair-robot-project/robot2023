package frc.team449.robot2023.subsystems.arm

import edu.wpi.first.math.geometry.Rotation2d
import frc.team449.robot2023.subsystems.arm.control.ArmPDController
import frc.team449.robot2023.subsystems.arm.control.ArmState
import frc.team449.robot2023.subsystems.arm.control.TwoJointArmFeedForward
import frc.team449.system.encoder.Encoder
import frc.team449.system.motor.WrappedMotor

/**
 * Simulate an arm to check logic of code or visualize trajectory
 */
class ArmSim(
  pivotMotor: WrappedMotor,
  jointMotor: WrappedMotor,
  feedForward: TwoJointArmFeedForward,
  controller: ArmPDController,
  pivotToJoint: Double,
  jointToEndEffector: Double
) : Arm(pivotMotor, jointMotor, feedForward, controller, pivotToJoint, jointToEndEffector) {

  private val pivotEncoder = Encoder.SimController(pivotMotor.encoder)
  private val jointEncoder = Encoder.SimController(jointMotor.encoder)
  override var state: ArmState
    get() =
      ArmState(
        Rotation2d(pivotEncoder.position),
        Rotation2d(jointEncoder.position),
        pivotEncoder.velocity,
        jointEncoder.velocity
      )
    set(value) {
      super.state = value
      pivotEncoder.velocity = desiredState.thetaVel
      jointEncoder.velocity = desiredState.betaVel
      pivotEncoder.position = desiredState.theta.radians
      jointEncoder.position = desiredState.beta.radians
    }

  override fun periodic() {
    super.periodic()
    pivotEncoder.position = pivotEncoder.position + pivotEncoder.velocity * .02
    jointEncoder.position = jointEncoder.position + jointEncoder.velocity * .02
    visual.setState(state)
  }
}
