package frc.team449.control.differential

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.control.DriveSubsystem
import frc.team449.system.AHRS
import frc.team449.system.encoder.Encoder
import frc.team449.system.motor.WrappedMotor
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log

/**
 * A differential drive with closed-loop velocity control using PID
 * @param makeVelPID Used to make two copies of the same PIDController for both sides of the
 * drivetrain
 */
class DifferentialDrive(
  private val leftLeader: WrappedMotor,
  private val rightLeader: WrappedMotor,
  private val ahrs: AHRS,
  private val feedforward: SimpleMotorFeedforward,
  makeVelPID: () -> PIDController,
  trackWidth: Double,
  val maxLinearSpeed: Double
) : DriveSubsystem, SubsystemBase(), Loggable {
  /**
   * The kinematics used to convert [DifferentialDriveWheelSpeeds] to [ChassisSpeeds]
   */
  val kinematics = DifferentialDriveKinematics(trackWidth)

  /** Odometry to keep track of where the robot is */
  private val odometry = DifferentialDriveOdometry(ahrs.heading)

  /** Velocity PID controller for left side */
  //  @Config.PIDController() //TODO causing casting errors
  private val leftPID = makeVelPID()

  /** Velocity PID controller for right side */
  //  @Config.PIDController()
  private val rightPID = makeVelPID()

  @Log.ToString
  private var desiredSpeeds = DifferentialDriveWheelSpeeds(0.0, 0.0)

  /**
   * Convert from x, y, rotation to left and right speeds
   *
   * @param desiredSpeeds The [ChassisSpeeds] desired for the drive
   */
  override fun set(desiredSpeeds: ChassisSpeeds) {
    val wheelSpeeds = kinematics.toWheelSpeeds(desiredSpeeds)
    wheelSpeeds.desaturate(this.maxLinearSpeed)
    val leftVel = wheelSpeeds.leftMetersPerSecond
    val rightVel = wheelSpeeds.rightMetersPerSecond
    leftLeader.setVoltage(
      feedforward.calculate(leftVel) + leftPID.calculate(leftLeader.velocity, leftVel)
    )
    rightLeader.setVoltage(
      feedforward.calculate(rightVel) + rightPID.calculate(rightLeader.velocity, rightVel)
    )
  }

  @get:Log.ToString
  override val heading: Rotation2d
    get() = ahrs.heading

  @get:Log.ToString
  override var pose: Pose2d
    get() = this.odometry.poseMeters
    set(pose) {
      leftLeader.encoder.resetPosition(0.0)
      rightLeader.encoder.resetPosition(0.0)
      ahrs.heading = pose.rotation
      this.odometry.resetPosition(pose, ahrs.heading)
    }

  override fun stop() {
    this.desiredSpeeds = DifferentialDriveWheelSpeeds(0.0, 0.0)
  }

  /** Periodically update the odometry */
  override fun periodic() {
    this.odometry.update(this.heading, this.leftLeader.position, this.rightLeader.position)
  }

  /**
   * Used for simulating a drivetrain
   */
  class SimController(
    private val realDrive: DifferentialDrive,
    private val driveSim: DifferentialDrivetrainSim,
    private val gyroSim: AHRS.SimController
  ) : DriveSubsystem.SimController {
    private val leftEncSim = Encoder.SimController(realDrive.leftLeader.encoder)
    private val rightEncSim = Encoder.SimController(realDrive.rightLeader.encoder)

    private var lastTime = Timer.getFPGATimestamp()

    override fun update() {
      driveSim.setInputs(realDrive.leftLeader.lastVoltage, realDrive.rightLeader.lastVoltage)

      val currTime = Timer.getFPGATimestamp()
      driveSim.update(currTime - lastTime)

      leftEncSim.velocity = driveSim.leftVelocityMetersPerSecond
      rightEncSim.velocity = driveSim.rightVelocityMetersPerSecond
      leftEncSim.position = driveSim.leftPositionMeters
      rightEncSim.position = driveSim.rightPositionMeters

      gyroSim.fusedHeading = driveSim.heading.degrees

      this.lastTime = currTime
    }

    override fun getCurrentDraw() = driveSim.currentDrawAmps
  }
}
