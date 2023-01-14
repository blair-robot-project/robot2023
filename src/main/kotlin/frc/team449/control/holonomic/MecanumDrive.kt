package frc.team449.control.holonomic

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.MecanumDriveKinematics
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.robot2022.drive.MecanumConstants
import frc.team449.robot2022.vision.VisionConstants
import frc.team449.system.AHRS
import frc.team449.system.VisionCamera
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log

/**
 * @param frontLeftMotor the front left motor
 * @param frontRightMotor the front right motor
 * @param backLeftMotor the back left motor
 * @param backRightMotor the back right motor
 * @param frontLeftLocation the offset of the front left wheel to the center of the robot
 * @param frontRightLocation the offset of the front right wheel to the center of the robot
 * @param backLeftLocation the offset of the back left wheel to the center of the robot
 * @param backRightLocation the offset of the back right wheel to the center of the robot
 * @param maxLinearSpeed the maximum translation speed of the chassis.
 * @param maxRotSpeed the maximum rotation speed of the chassis
 * @param feedForward the SimpleMotorFeedforward for mecanum
 * @param controller the PIDController for the robot
 */
open class MecanumDrive(
  private val frontLeftMotor: WrappedMotor,
  private val frontRightMotor: WrappedMotor,
  private val backLeftMotor: WrappedMotor,
  private val backRightMotor: WrappedMotor,
  frontLeftLocation: Translation2d,
  frontRightLocation: Translation2d,
  backLeftLocation: Translation2d,
  backRightLocation: Translation2d,
  private val ahrs: AHRS,
  override val maxLinearSpeed: Double,
  override val maxRotSpeed: Double,
  private val feedForward: SimpleMotorFeedforward,
  private val controller: () -> PIDController,
  private val cameras: List<VisionCamera> = mutableListOf()
) : HolonomicDrive, SubsystemBase(), Loggable {

  private val flController = controller()
  private val frController = controller()
  private val blController = controller()
  private val brController = controller()

  private var lastTime = Timer.getFPGATimestamp()

  val kinematics = MecanumDriveKinematics(
    frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
  )

  private val poseEstimator = MecanumDrivePoseEstimator(
    kinematics,
    ahrs.heading,
    getPositions(),
    DriveConstants.INITIAL_POSE,
    MatBuilder(Nat.N3(), Nat.N1()).fill(.005, .005, .0005), // [x, y, theta] other estimates
    MatBuilder(Nat.N3(), Nat.N1()).fill(.005, .005, .0005) // [x, y, theta] vision estimates
  )

  override var pose: Pose2d
    @Log.ToString(name = "Pose")
    get() {
      return this.poseEstimator.estimatedPosition
    }
    set(value) {
      this.poseEstimator.resetPosition(ahrs.heading, getPositions(), value)
    }

  @Log.ToString(name = "Desired Mecanum Speeds")
  private var desiredWheelSpeeds = MecanumDriveWheelSpeeds()

  override fun set(desiredSpeeds: ChassisSpeeds) {
    desiredWheelSpeeds = kinematics.toWheelSpeeds(desiredSpeeds)
    desiredWheelSpeeds.desaturate(MecanumConstants.MAX_ATTAINABLE_WHEEL_SPEED)
  }

  override fun stop() {
    this.set(ChassisSpeeds(0.0, 0.0, 0.0))
  }

  override fun periodic() {
    val currTime = Timer.getFPGATimestamp()

    val frontLeftPID = flController.calculate(frontLeftMotor.velocity, desiredWheelSpeeds.frontLeftMetersPerSecond)
    val frontRightPID = frController.calculate(frontRightMotor.velocity, desiredWheelSpeeds.frontRightMetersPerSecond)
    val backLeftPID = blController.calculate(backLeftMotor.velocity, desiredWheelSpeeds.rearLeftMetersPerSecond)
    val backRightPID = brController.calculate(backRightMotor.velocity, desiredWheelSpeeds.rearRightMetersPerSecond)

    val frontLeftFF = feedForward.calculate(
      desiredWheelSpeeds.frontLeftMetersPerSecond
    )
    val frontRightFF = feedForward.calculate(
      desiredWheelSpeeds.frontRightMetersPerSecond
    )
    val backLeftFF = feedForward.calculate(
      desiredWheelSpeeds.rearLeftMetersPerSecond
    )
    val backRightFF = feedForward.calculate(
      desiredWheelSpeeds.rearRightMetersPerSecond
    )

    frontLeftMotor.setVoltage(frontLeftPID + frontLeftFF)
    frontRightMotor.setVoltage(frontRightPID + frontRightFF)
    backLeftMotor.setVoltage(backLeftPID + backLeftFF)
    backRightMotor.setVoltage(backRightPID + backRightFF)

    if (cameras.isNotEmpty()) localize()

    this.poseEstimator.update(
      ahrs.heading,
      getPositions()
    )

    lastTime = currTime
  }

  /**
   * @return the position readings of the wheels bundled into one object (meters)
   */
  private fun getPositions(): MecanumDriveWheelPositions =
    MecanumDriveWheelPositions(
      frontLeftMotor.position,
      frontRightMotor.position,
      backLeftMotor.position,
      backRightMotor.position
    )

  /**
   * @return the velocity readings of the wheels bundled into one object (meters/s)
   */
  private fun getSpeeds(): MecanumDriveWheelSpeeds =
    MecanumDriveWheelSpeeds(
      frontLeftMotor.velocity,
      frontRightMotor.velocity,
      backLeftMotor.velocity,
      backRightMotor.velocity
    )

  private fun localize() {
    for (camera in cameras) {
      if (camera.hasTarget()) {
        poseEstimator.addVisionMeasurement(
          camera.camPose().toPose2d(),
          camera.timestamp()
        )
      }
    }
  }

  companion object {
    /** Create a new Mecanum Drive from DriveConstants */
    fun createMecanum(ahrs: AHRS): MecanumDrive {
      return MecanumDrive(
        createSparkMax("frontLeft", MecanumConstants.DRIVE_MOTOR_FL, NEOEncoder.creator(MecanumConstants.DRIVE_UPR, MecanumConstants.DRIVE_GEARING)),
        createSparkMax("frontRight", MecanumConstants.DRIVE_MOTOR_FR, NEOEncoder.creator(MecanumConstants.DRIVE_UPR, MecanumConstants.DRIVE_GEARING), inverted = true),
        createSparkMax("backLeft", MecanumConstants.DRIVE_MOTOR_BL, NEOEncoder.creator(MecanumConstants.DRIVE_UPR, MecanumConstants.DRIVE_GEARING)),
        createSparkMax("backRight", MecanumConstants.DRIVE_MOTOR_BR, NEOEncoder.creator(MecanumConstants.DRIVE_UPR, MecanumConstants.DRIVE_GEARING), inverted = true),
        Translation2d(MecanumConstants.WHEELBASE / 2, MecanumConstants.TRACKWIDTH / 2),
        Translation2d(MecanumConstants.WHEELBASE / 2, -MecanumConstants.TRACKWIDTH / 2),
        Translation2d(-MecanumConstants.WHEELBASE / 2, MecanumConstants.TRACKWIDTH / 2),
        Translation2d(-MecanumConstants.WHEELBASE / 2, -MecanumConstants.TRACKWIDTH / 2),
        ahrs,
        DriveConstants.MAX_LINEAR_SPEED,
        DriveConstants.MAX_ROT_SPEED,
        SimpleMotorFeedforward(MecanumConstants.DRIVE_KS, MecanumConstants.DRIVE_KV, MecanumConstants.DRIVE_KA),
        { PIDController(MecanumConstants.DRIVE_KP, MecanumConstants.DRIVE_KI, MecanumConstants.DRIVE_KD) },
        VisionConstants.CAMERAS
      )
    }
  }
}
