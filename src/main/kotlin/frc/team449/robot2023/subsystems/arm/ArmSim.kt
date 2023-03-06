package frc.team449.robot2023.subsystems.arm

import edu.wpi.first.math.geometry.Rotation2d
import frc.team449.robot2023.subsystems.arm.control.ArmPDController
import frc.team449.robot2023.subsystems.arm.control.ArmState
import frc.team449.robot2023.subsystems.arm.control.TwoJointArmFeedForward
import frc.team449.system.encoder.Encoder
import frc.team449.system.encoder.QuadEncoder
import frc.team449.system.motor.WrappedMotor

/**
 * Simulate an arm to check logic of code or visualize trajectory
 */
class ArmSim(
  firstJoint: WrappedMotor,
  secondJoint: WrappedMotor,
  firstJointEncoder: QuadEncoder,
  secondJointEncoder: QuadEncoder,
  feedForward: TwoJointArmFeedForward,
  controller: ArmPDController,
  firstToSecondJoint: Double,
  secondJointToEndEffector: Double
) : Arm(firstJoint, secondJoint, firstJointEncoder, secondJointEncoder, feedForward, controller, firstToSecondJoint, secondJointToEndEffector) {

  private val firstJointEnc = Encoder.SimController(firstJoint.encoder)
  private val secondJointEnc = Encoder.SimController(secondJoint.encoder)
  override var state: ArmState
    get() =
      ArmState(
        Rotation2d(firstJointEnc.position),
        Rotation2d(secondJointEnc.position),
        firstJointEnc.velocity,
        secondJointEnc.velocity
      )
    set(value) {
      desiredState = value
    }

  override fun periodic() {
    firstJointEnc.velocity = desiredState.thetaVel
    secondJointEnc.velocity = desiredState.betaVel
    firstJointEnc.position = desiredState.theta.radians
    secondJointEnc.position = desiredState.beta.radians
    firstJointEnc.position = firstJointEnc.position + firstJointEnc.velocity * .02
    secondJointEnc.position = secondJointEnc.position + secondJointEnc.velocity * .02
    visual.setState(state, desiredState)
  }
}
