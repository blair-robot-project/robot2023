package frc.team449.control.differential

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import frc.team449.control.DriveSubsystem
import frc.team449.system.AHRS
import frc.team449.system.motor.WrappedMotor
import io.github.oblarg.oblog.annotations.Config
import io.github.oblarg.oblog.annotations.Log

/** A differential drive with closed-loop velocity control using PID
 * @param makeVelPID Used to make two copies of the same PIDController for both sides of the drivetrain
 */
class DifferentialDrive(
  private val leftLeader: WrappedMotor,
  private val rightLeader: WrappedMotor,
  private val ahrs: AHRS,
  private val feedforward: SimpleMotorFeedforward,
  makeVelPID: () -> PIDController,
  private val trackWidth: Double,
  val maxLinearSpeed: Double
) : DriveSubsystem {
  /**
   * The kinematics used to convert {@link DifferentialDriveWheelSpeeds} to {@link ChassisSpeeds}
   */
  val kinematics = DifferentialDriveKinematics(trackWidth)
  /** Odometry to keep track of where the robot is */
  private val odometry = DifferentialDriveOdometry(ahrs.heading)

  /** Velocity PID controller for left side */
  @Config.PIDController
  private val leftPID = makeVelPID()
  /** Velocity PID controller for right side */
  @Config.PIDController
  private val rightPID = makeVelPID()

  val maxRotSpeed = 2 * maxLinearSpeed / trackWidth

  @Log.ToString
  private var desiredSpeeds = DifferentialDriveWheelSpeeds(0.0, 0.0)

  /**
   * Convert from x, y, rotation to left and right speeds
   *
   * @param desiredSpeeds The {@link ChassisSpeeds} desired for the drive
   */
  override fun set(desiredSpeeds: ChassisSpeeds) {
    this.desiredSpeeds = kinematics.toWheelSpeeds(desiredSpeeds)
  }

  @get:Log
  override val heading: Rotation2d
    get() { return ahrs.heading }

  @get:Log
  override var pose: Pose2d
    get() {
      return this.odometry.getPoseMeters()
    }
    set(pose) {
      leftLeader.encoder.position = 0.0
      rightLeader.encoder.position = 0.0
      ahrs.heading = pose.getRotation()
      this.odometry.resetPosition(pose, ahrs.heading)
    }

  override fun stop() {
    this.desiredSpeeds = DifferentialDriveWheelSpeeds(0.0, 0.0)
  }

  /** Periodically update the odometry */
  override fun periodic() {
    val leftPosition = this.leftLeader.position
    val rightPosition = this.rightLeader.position
    this.odometry.update(this.heading, leftPosition, rightPosition)

    desiredSpeeds.desaturate(this.maxLinearSpeed)
    val leftVel = desiredSpeeds.leftMetersPerSecond
    val rightVel = desiredSpeeds.rightMetersPerSecond
    leftLeader.setVoltage(
      feedforward.calculate(leftVel) +
        leftPID.calculate(leftLeader.getVelocity(), leftVel)
    )
    rightLeader.setVoltage(
      feedforward.calculate(rightVel) +
        rightPID.calculate(rightLeader.getVelocity(), rightVel)
    )
  }
}
