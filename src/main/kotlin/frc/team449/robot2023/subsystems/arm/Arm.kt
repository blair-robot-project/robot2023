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
  val firstJointEncoder: QuadEncoder,
  val secondJointEncoder: QuadEncoder,
  private val feedForward: TwoJointArmFeedForward,
  @field:Log val controller: ArmPDController,
  firstToSecondJoint: Double,
  secondJointToEndEffector: Double
) : Loggable, SubsystemBase() {

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
  open val state: ArmState
    get() = ArmState(
      Rotation2d(MathUtil.inputModulus(firstJointEncoder.position, -PI, PI)),
      Rotation2d(MathUtil.inputModulus(secondJointEncoder.position, -PI, PI)),
      firstJointEncoder.velocity,
      secondJointEncoder.velocity
    )

  /**
   * The current state of the arm in [CartesianArmState]
   */
  @get:Log.ToString
  val coordinate: CartesianArmState
    get() = kinematics.toCartesian(state)

  fun setArmDesiredState(desiredState: ArmState) {
    val desState = ArmState(
      desiredState.theta,
      Rotation2d.fromDegrees(
        clamp(desiredState.beta.degrees, -156.8, 151.15)
      ),
      desiredState.thetaVel,
      desiredState.betaVel
    )
//     continue if state is same as last desired state
      if (desState == desiredState) {
        return
      }
    controller.reset()
    this.desiredState = desState
  }

  fun moveToState(desiredState: ArmState) {
    setArmDesiredState(desiredState)
    holdArm()
  }

  fun holdArm() {
    val ff = feedForward.calculate(state.static().matrix, true)
    val pid = controller.calculate(state.matrix, desiredState.matrix)
    val u = ff + pid
    firstJoint.setVoltage(u[0, 0])
    secondJoint.setVoltage(u[1, 0])
  }

  /**
   * Mitigate any speed on the joints
   */
  fun stop() {
    desiredState.thetaVel = 0.0
    desiredState.betaVel = 0.0
  }

  override fun periodic() {
    visual.setState(state, desiredState)
  }

  fun getClosestState(point: ArmState): ArmState? {
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

        ArmConstants.CUBE ->
          ArmPaths.stowCube

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
