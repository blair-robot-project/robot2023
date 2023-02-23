package frc.team449.robot2023.subsystems.arm

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.arm.ArmConstants
import frc.team449.system.motor.WrappedMotor
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log
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
  private val feedForward: TwoJointArmFeedForward,
  private val controller: ArmPDController,
  firstToSecondJoint: Double,
  secondJointToEndEffector: Double
) : Loggable, SubsystemBase() {

  /** visual of the arm as a Mechanism2d object */
  val visual = ArmVisual(
    firstToSecondJoint,
    secondJointToEndEffector
  )

  /** kinematics that converts between (x, y) <-> (theta, beta) coordinates */
  val kinematics = ArmKinematics(
    firstToSecondJoint,
    secondJointToEndEffector
  )

  /** desired arm state */
  @Log.ToString
  var desiredState = ArmState(
    Rotation2d(1.63),
    Rotation2d(-2.51)
  )

  /**
   * the current state of the arm in [ArmState]
   */
  @get:Log.ToString
  open var state: ArmState
    get() = ArmState(
      Rotation2d(firstJoint.position),
      Rotation2d(secondJoint.position),
      firstJoint.velocity,
      secondJoint.velocity
    )
    set(state) {
      desiredState = state
    }

  /**
   * The current state of the arm in [CartesianArmState]
   */
  @get:Log.ToString
  var coordinate: CartesianArmState
    get() = kinematics.toCartesian(state)
    set(coordinate) {
      /**
       * null safety, return if the angular state is null
       */
      val angular = kinematics.toAngularState(coordinate, state) ?: return
      state = angular
    }

  /**
   * Mitigate any speed on the joints
   */
  fun stop() {
    desiredState.thetaVel = 0.0
    desiredState.betaVel = 0.0
  }
  override fun periodic() {
    val ff = feedForward.calculate(desiredState.matrix, true)
    val pid = controller.calculate(state.matrix, desiredState.matrix)
    val u = ff + pid
    firstJoint.setVoltage(u[0, 0])
    secondJoint.setVoltage(u[1, 0])
    visual.setState(state)
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
    return sqrt(
      (state1.theta.degrees - state2.theta.degrees).pow(2.0) + (state1.beta.degrees - state2.beta.degrees).pow(2.0)
    )
  }
  fun chooseTraj(endpoint: ArmState): ArmTrajectory? {
    val startPoint = getClosestState(this.state)
    if (endpoint == startPoint) return null
    if (startPoint == ArmConstants.STOW) {
      return when (endpoint) {
        ArmConstants.HIGH ->
          ArmPaths.STOW_HIGH
        ArmConstants.MID ->
          ArmPaths.STOW_MID
        ArmConstants.LOW ->
          ArmPaths.STOW_LOW
        else ->
          ArmPaths.STOW_CONE
      }
    } else {
      return when (startPoint) {
        ArmConstants.HIGH ->
          ArmPaths.HIGH_STOW
        ArmConstants.MID ->
          ArmPaths.MID_STOW
        ArmConstants.LOW ->
          ArmPaths.LOW_STOW
        else ->
          ArmPaths.CONE_STOW
      }
    }
  }
}
