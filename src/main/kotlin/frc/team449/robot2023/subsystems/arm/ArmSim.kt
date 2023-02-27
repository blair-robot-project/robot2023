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
  firstJoint: WrappedMotor,
  secondJoint: WrappedMotor,
  feedForward: TwoJointArmFeedForward,
  controller: ArmPDController,
  firstToSecondJoint: Double,
  secondJointToEndEffector: Double
) : Arm(firstJoint, secondJoint, feedForward, controller, firstToSecondJoint, secondJointToEndEffector) {

  private val firstJointEncoder = Encoder.SimController(firstJoint.encoder)
  private val secondJointEncoder = Encoder.SimController(secondJoint.encoder)
  override var state: ArmState
    get() =
      ArmState(
        Rotation2d(firstJointEncoder.position),
        Rotation2d(secondJointEncoder.position),
        firstJointEncoder.velocity,
        secondJointEncoder.velocity
      )
    set(value) {
      super.state = value
    }

  override fun periodic() {
    firstJointEncoder.velocity = desiredState.thetaVel
    secondJointEncoder.velocity = desiredState.betaVel
    firstJointEncoder.position = desiredState.theta.radians
    secondJointEncoder.position = desiredState.beta.radians
    super.periodic()
    firstJointEncoder.position = firstJointEncoder.position + firstJointEncoder.velocity * .02
    secondJointEncoder.position = secondJointEncoder.position + secondJointEncoder.velocity * .02
    visual.setState(state, desiredState)
  }
}
