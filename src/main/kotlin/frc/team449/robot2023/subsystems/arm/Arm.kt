package frc.team449.robot2023.subsystems.arm

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.subsystems.arm.control.ArmKinematics
import frc.team449.robot2023.subsystems.arm.control.ArmPDController
import frc.team449.robot2023.subsystems.arm.control.ArmState
import frc.team449.robot2023.subsystems.arm.control.CartesianArmState
import frc.team449.robot2023.subsystems.arm.control.TwoJointArmFeedForward
import frc.team449.system.motor.WrappedMotor
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log

/**
 * Controllable two-jointed arm
 * @param pivotMotor main motor that moves the whole arm
 * @param jointMotor main motor at the joint that moves the second segment of the arm
 * @param feedForward the calculator for voltages based on a desired state
 * @param pivotToJoint length from the pivot motor to joint motor in METERS
 * @param jointToEndEffector length from the joint motor to the end-effector of the arm in METERS
 */
open class Arm(
  private val pivotMotor: WrappedMotor,
  private val jointMotor: WrappedMotor,
  private val feedForward: TwoJointArmFeedForward,
  private val controller: ArmPDController,
  pivotToJoint: Double,
  jointToEndEffector: Double
) : Loggable, SubsystemBase() {
  init {
    pivotMotor.encoder.resetPosition(0.0)
    jointMotor.encoder.resetPosition(0.0)
  }
  /** visual of the arm as a Mechanism2d object */
  val visual = ArmVisual(
    pivotToJoint,
    jointToEndEffector
  )

  /** kinematics that converts between (x, y) <-> (theta, beta) coordinates */
  val kinematics = ArmKinematics(
    pivotToJoint,
    jointToEndEffector
  )

  /** desired arm state */
  @Log.ToString
  var desiredState = ArmState(
    Rotation2d(pivotMotor.position),
    Rotation2d(jointMotor.position)
  )

  /**
   * the current state of the arm in [ArmState]
   */
  @get:Log.ToString
  open var state: ArmState
    get() = ArmState(
      Rotation2d(pivotMotor.position),
      Rotation2d(jointMotor.position),
      pivotMotor.velocity,
      jointMotor.velocity
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
    val ff = feedForward.calculate(desiredState.matrix)
    val pid = controller.calculate(state.matrix, desiredState.matrix)
    val u = ff + pid
    pivotMotor.setVoltage(u[0, 0])
    jointMotor.setVoltage(u[1, 0])
    visual.setState(state)
  }
}
