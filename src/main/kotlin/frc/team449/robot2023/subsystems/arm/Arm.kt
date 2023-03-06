package frc.team449.robot2023.subsystems.arm

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.arm.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmKinematics
import frc.team449.robot2023.subsystems.arm.control.ArmPDController
import frc.team449.robot2023.subsystems.arm.control.ArmState
import frc.team449.robot2023.subsystems.arm.control.ArmTrajectory
import frc.team449.robot2023.subsystems.arm.control.CartesianArmState
import frc.team449.robot2023.subsystems.arm.control.TwoJointArmFeedForward
import frc.team449.system.encoder.QuadEncoder
import frc.team449.system.motor.WrappedMotor
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log
import kotlin.math.PI
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Controllable two-jointed arm
 * @param firstJoint main motor that moves the whole arm
 * @param secondJoint main motor at the joint that moves the second segment of the arm
 * @param feedForward the calculator for voltages based on a desired state
 * @param firstToSecondJoint length from the pivot motor to joint motor in METERS
 * @param secondJointToEndEffector length from the joint motor to the end-effector of the arm in METERS
 */
open class Arm(
  val firstJoint: WrappedMotor,
  val secondJoint: WrappedMotor,
  private val firstJointEncoder: QuadEncoder,
  private val secondJointEncoder: QuadEncoder,
  private val feedForward: TwoJointArmFeedForward,
  @field:Log val controller: ArmPDController,
  firstToSecondJoint: Double,
  secondJointToEndEffector: Double,
  private val numSamples: Int = 150
) : Loggable, SubsystemBase() {

  /** PWM signal measurement samples */
  private var firstJointSamples = mutableListOf<Double>()
  private var secondJointSamples = mutableListOf<Double>()
  @Log
  private var calibrated = false
  /** visual of the arm as a Mechanism2d object */
  val visual = ArmVisual(
    firstToSecondJoint,
    secondJointToEndEffector,
    "Arm Visual :)"
  )

  /** kinematics that converts between (x, y) <-> (theta, beta) coordinates */
  val kinematics = ArmKinematics(
    firstToSecondJoint,
    secondJointToEndEffector
  )

  /** desired arm state */
  @Log.ToString
  var desiredState = ArmConstants.STOW

  /**
   * the current state of the arm in [ArmState]
   */
  @get:Log.ToString
  open var state: ArmState
    get() = ArmState(
      Rotation2d(MathUtil.inputModulus(firstJointEncoder.position, -PI, PI)),
      Rotation2d(MathUtil.inputModulus(secondJointEncoder.position, -PI, PI)),
      firstJointEncoder.velocity,
      secondJointEncoder.velocity
    )
    set(state) {
//    cap q2 between its hard limits
      state.beta = Rotation2d.fromDegrees(
        clamp(state.beta.degrees, -156.8, 151.15)
      )
      // continue if state is same as last desired state
      if (state == desiredState) return
      controller.reset()
      desiredState = state
    }

  /**
   * The current state of the arm in [CartesianArmState]
   */
  @get:Log.ToString
  val coordinate: CartesianArmState
    get() = kinematics.toCartesian(state)

  /**
   * Mitigate any speed on the joints
   */
  fun stop() {
    desiredState.thetaVel = 0.0
    desiredState.betaVel = 0.0
  }

  /**
   * **!!! THE ARM BETTER BE STATIONARY WHEN DOING THIS !!!**
   * Quadrature is better at measuring position but cannot measure absolute
   * Read 500 PWM signals and take the high pulse readings to apply initial measurement for quad
   */
  fun resetQuadrature() {
    firstJointSamples.removeAll { true }
    secondJointSamples.removeAll { true }
    calibrated = false
  }
  override fun periodic() {
    if (firstJointSamples.size < numSamples) {
      firstJointSamples.add(firstJoint.position)
      secondJointSamples.add(secondJoint.position)
      return
    }
    if (firstJointSamples.size == numSamples && !calibrated) {
      // update encoder reading of quad
      firstJointSamples.sort()
      secondJointSamples.sort()
      val firstJointPos = firstJointSamples[(firstJointSamples.size * .9).toInt()]
      val secondJointPos = secondJointSamples[(secondJointSamples.size * .9).toInt()]
      firstJointEncoder.resetPosition(firstJointPos)
      secondJointEncoder.resetPosition(secondJointPos)
      calibrated = true
      println("***** Finished Calibrating Quadrature reading *****")
    }
    val ff = feedForward.calculate(desiredState.matrix, false)
    val pid = controller.calculate(state.matrix, desiredState.matrix)
    val u = ff + pid
    firstJoint.setVoltage(u[0, 0])
    secondJoint.setVoltage(u[1, 0])
    visual.setState(state, desiredState)
  }

  private fun getClosestState(point: ArmState): ArmState? {
    var closestState: ArmState? = null
    var closestDistance = Double.MAX_VALUE

    for (state in ArmConstants.STATES) {
      val distanceToState = distanceBetweenStates(point, state)

      if (distanceToState < closestDistance) {
        closestDistance = distanceToState
        closestState = state
      }
    }

    return closestState
  }

  private fun distanceBetweenStates(state1: ArmState, state2: ArmState): Double {
    val coordinate1 = kinematics.toCartesian(state1)
    val coordinate2 = kinematics.toCartesian(state2)
    return sqrt(
      (coordinate1.x - coordinate2.x).pow(2.0) + (coordinate1.z - coordinate2.z).pow(2.0)
    )
  }
  fun chooseTraj(endpoint: ArmState): ArmTrajectory? {
    val startPoint = getClosestState(this.desiredState)
    if (endpoint == startPoint) return null
    if (startPoint == ArmConstants.STOW) {
      return when (endpoint) {
        ArmConstants.HIGH ->
          ArmPaths.STOW_HIGH
        ArmConstants.MID ->
          ArmPaths.STOW_MID
        ArmConstants.LOW ->
          ArmPaths.STOW_LOW
        ArmConstants.CONE ->
          ArmPaths.STOW_CONE
        else ->
          ArmPaths.STOW_CUBE
      }
    } else {
      return when (startPoint) {
        ArmConstants.HIGH ->
          ArmPaths.HIGH_STOW
        ArmConstants.MID ->
          ArmPaths.MID_STOW
        ArmConstants.LOW ->
          ArmPaths.LOW_STOW
        ArmConstants.CONE ->
          ArmPaths.CONE_STOW
        else ->
          ArmPaths.CUBE_STOW
      }
    }
  }
}
