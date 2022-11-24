package frc.team449.control.holonomic

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.MecanumDriveKinematics
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.system.AHRS
import frc.team449.system.VisionCamera
// import frc.team449.system.VisionCamera
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
  val ahrs: AHRS,
  override val maxLinearSpeed: Double,
  override val maxRotSpeed: Double,
  private val feedForward: SimpleMotorFeedforward,
  private val controller: () -> PIDController,
  private val cameras: MutableList<VisionCamera> = mutableListOf()
) : HolonomicDrive, SubsystemBase(), Loggable {
  init {
    ahrs.calibrate()
    ahrs.reset()
  }
  private val flController = controller()
  private val frController = controller()
  private val blController = controller()
  private val brController = controller()

  // 10.500 x, 10.713 y (outreach 2022) (in.) (top right) (y is horizontal axis)

  val kinematics = MecanumDriveKinematics(
    frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
  )

  private val poseEstimator = MecanumDrivePoseEstimator(
    ahrs.heading,
    Pose2d(),
    kinematics,
    MatBuilder(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.01),
    MatBuilder(Nat.N1(), Nat.N1()).fill(0.01),
    MatBuilder(Nat.N3(), Nat.N1()).fill(.005, .005, .0005)
  )

  override val heading: Rotation2d
    get() {
      return ahrs.heading
    }

  override var pose: Pose2d
    @Log.ToString
    get() {
      return this.poseEstimator.estimatedPosition
    }
    set(value) {
      this.poseEstimator.resetPosition(value, heading)
    }

  @Log.ToString
  private var desiredWheelSpeeds = MecanumDriveWheelSpeeds()

  override fun set(desiredSpeeds: ChassisSpeeds) {
    desiredWheelSpeeds = kinematics.toWheelSpeeds(desiredSpeeds)
    desiredWheelSpeeds.desaturate(DriveConstants.MAX_ATTAINABLE_WHEEL_SPEED)
  }

  override fun stop() {
    this.set(ChassisSpeeds(0.0, 0.0, 0.0))
  }

  override fun periodic() {

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
      heading,
      MecanumDriveWheelSpeeds(
        frontLeftMotor.velocity,
        frontRightMotor.velocity,
        backLeftMotor.velocity,
        backRightMotor.velocity
      )
    )
  }

  fun addCamera(camera: VisionCamera) {
    cameras.add(camera)
  }

  private fun localize() {
    for (camera in cameras) {
      if (camera.hasTarget()) {
        poseEstimator.addVisionMeasurement(
          camera.camPose(Pose3d(Transform3d())).toPose2d(),
          camera.timestamp()
        )
      }
    }
  }

  companion object {
    /** Create a new Mecanum Drive from DriveConstants */
    fun createMecanum(ahrs: AHRS): MecanumDrive {
      return MecanumDrive(
        createSparkMax("frontLeft", DriveConstants.DRIVE_MOTOR_FL, NEOEncoder.creator(DriveConstants.DRIVE_UPR, DriveConstants.DRIVE_GEARING)),
        createSparkMax("frontRight", DriveConstants.DRIVE_MOTOR_FR, NEOEncoder.creator(DriveConstants.DRIVE_UPR, DriveConstants.DRIVE_GEARING), inverted = true),
        createSparkMax("backLeft", DriveConstants.DRIVE_MOTOR_BL, NEOEncoder.creator(DriveConstants.DRIVE_UPR, DriveConstants.DRIVE_GEARING)),
        createSparkMax("backRight", DriveConstants.DRIVE_MOTOR_BR, NEOEncoder.creator(DriveConstants.DRIVE_UPR, DriveConstants.DRIVE_GEARING), inverted = true),
        Translation2d(DriveConstants.WHEELBASE / 2, DriveConstants.TRACKWIDTH / 2),
        Translation2d(DriveConstants.WHEELBASE / 2, -DriveConstants.TRACKWIDTH / 2),
        Translation2d(-DriveConstants.WHEELBASE / 2, DriveConstants.TRACKWIDTH / 2),
        Translation2d(-DriveConstants.WHEELBASE / 2, -DriveConstants.TRACKWIDTH / 2),
        ahrs,
        DriveConstants.MAX_LINEAR_SPEED,
        DriveConstants.MAX_ROT_SPEED,
        SimpleMotorFeedforward(DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV, DriveConstants.DRIVE_KA),
        { PIDController(DriveConstants.DRIVE_KP, DriveConstants.DRIVE_KI, DriveConstants.DRIVE_KD) }
      )
    }

    fun simDrive(ahrs: AHRS): MecanumDrive {
      return MecanumSim(ahrs)
    }
  }
}
