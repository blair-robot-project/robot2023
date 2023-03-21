package frc.team449.robot2023.subsystems.arm

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmEncoder
import frc.team449.robot2023.subsystems.arm.control.ArmKinematics
import frc.team449.robot2023.subsystems.arm.control.ArmPDController
import frc.team449.robot2023.subsystems.arm.control.ArmState
import frc.team449.robot2023.subsystems.arm.control.ArmTrajectory
import frc.team449.robot2023.subsystems.arm.control.CartesianArmState
import frc.team449.robot2023.subsystems.arm.control.TwoJointArmFeedForward
import frc.team449.system.encoder.QuadEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax
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
    if (endpoint == startPoint) {
      this.desiredState = endpoint
      return null
    }
    return if (startPoint == ArmConstants.STOW) {
      when (endpoint) {
        ArmConstants.SINGLE ->
          ArmPaths.stowSingle
        ArmConstants.DOUBLE ->
          ArmPaths.stowDouble
        ArmConstants.CONE ->
          ArmPaths.stowCone
        ArmConstants.MID ->
          ArmPaths.stowMid
        else ->
          ArmPaths.stowHigh
      }
    } else {
      when (startPoint) {
        ArmConstants.SINGLE ->
          ArmPaths.singleStow
        ArmConstants.DOUBLE ->
          ArmPaths.doubleStow
        ArmConstants.CONE ->
          ArmPaths.coneStow
        ArmConstants.CUBE ->
          ArmPaths.cubeStow
        ArmConstants.MID ->
          ArmPaths.midStow
        else ->
          ArmPaths.highStow
      }
    }
  }

  companion object {
    fun createArm(): Arm {
      val firstJointMotor = createSparkMax(
        "First Joint Motor",
        ArmConstants.FIRST_MOTOR_ID1,
        ArmEncoder.creator(
          ArmConstants.FIRST_ENCODER_CHAN,
          ArmConstants.FIRST_ENCODER_OFFSET,
          true
        ),
        slaveSparks = mapOf(
          ArmConstants.FIRST_MOTOR_ID2 to true
        ),
        currentLimit = ArmConstants.FIRST_JOINT_CURR_LIM,
        inverted = true,
        enableBrakeMode = true
      )

      val secondJointMotor = createSparkMax(
        "Second Joint Motor",
        ArmConstants.SECOND_MOTOR_ID,
        ArmEncoder.creator(
          ArmConstants.SECOND_ENCODER_CHAN,
          ArmConstants.SECOND_ENCODER_OFFSET,
          inverted = true
        ),
        currentLimit = ArmConstants.SECOND_JOINT_CURR_LIM,
        enableBrakeMode = true
      )

      val firstJointEncoder = QuadEncoder(
        "First joint quad",
        ArmConstants.FIRSTJ_QUAD_ENCODER,
        1024,
        2 * PI,
        1.0
      )

      val secondJointEncoder = QuadEncoder(
        "Second joint quad",
        ArmConstants.SECONDJ_QUAD_ENCODER,
        1024,
        2 * PI,
        1.0
      )

      return Arm(
        firstJointMotor,
        secondJointMotor,
        firstJointEncoder,
        secondJointEncoder,
        TwoJointArmFeedForward.createFromConstants(),
        ArmPDController(
          ArmConstants.kP1,
          ArmConstants.kP2,
          ArmConstants.kD1,
          ArmConstants.kD2,
          ArmConstants.kI1,
          ArmConstants.kI2,
          ArmConstants.kErrDeadband
        ),
        ArmConstants.LENGTH_1,
        ArmConstants.LENGTH_2
      )
    }
  }
}
