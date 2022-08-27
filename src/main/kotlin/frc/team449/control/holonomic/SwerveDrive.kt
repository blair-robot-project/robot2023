package frc.team449.control.holonomic

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.AHRS
import frc.team449.system.motor.WrappedMotor
import io.github.oblarg.oblog.annotations.Log

open class SwerveDrive(
  private val modules: List<SwerveModule>,
  val ahrs: AHRS,
  override val maxLinearSpeed: Double,
  override val maxRotSpeed: Double
) : SubsystemBase(), HolonomicDrive {
  init {
    // Zero out the gyro
    ahrs.reset()
  }
  private val kinematics = SwerveDriveKinematics(
    *this.modules
      .map { it.location }.toTypedArray()
  )
  private val odometry = SwerveDriveOdometry(this.kinematics, ahrs.heading)

  @Log.ToString
  var desiredSpeeds = ChassisSpeeds()

  override fun set(desiredSpeeds: ChassisSpeeds) {
    this.desiredSpeeds = desiredSpeeds
  }

  override val heading: Rotation2d
    @Log.ToString
    get() {
      return ahrs.heading
    }

  override var pose: Pose2d
    @Log.ToString
    get() {
      return this.odometry.poseMeters
    }
    set(pose) {
      this.odometry.resetPosition(pose, ahrs.heading)
    }

  override fun stop() {
    this.set(ChassisSpeeds(0.0, 0.0, 0.0))
  }

  override fun periodic() {
    val desiredModuleStates =
      this.kinematics.toSwerveModuleStates(this.desiredSpeeds)

    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredModuleStates,
      this.maxLinearSpeed
    )

    for (i in this.modules.indices) {
      this.modules[i].state = desiredModuleStates[i]
    }

    this.odometry.update(
      ahrs.heading,
      *this.modules
        .map { it.state }.toTypedArray()
    )

    for (module in modules)
      module.update()
  }

  companion object {
    /**
     * Create a square swerve drivetrain
     *
     * @param ahrs Gyro used for robot heading
     * @param maxLinearSpeed Max speed (m/s) at which the robot can translate
     * @param maxRotSpeed Max speed (rad/s) at which the robot can turn in place
     * @param frontLeftDriveMotor
     * @param frontRightDriveMotor
     * @param backLeftDriveMotor
     * @param backRightDriveMotor
     * @param frontLeftTurnMotor
     * @param frontRightTurnMotor
     * @param backLeftTurnMotor
     * @param backRightTurnMotor
     * @param frontLeftLocation Location of the front left module
     * @param driveMotorController Supplier to make copies of the same driving PID controllers for all
     *     the modules
     * @param turnMotorController Supplier to make copies of the same turning PID controllers for all
     *     the modules
     * @param driveFeedforward Driving feedforward for all the modules
     * @param turnFeedforward Turning feedforward for all the modules
     */
    fun squareDrive(
      ahrs: AHRS,
      maxLinearSpeed: Double,
      maxRotSpeed: Double,
      frontLeftDriveMotor: WrappedMotor,
      frontRightDriveMotor: WrappedMotor,
      backLeftDriveMotor: WrappedMotor,
      backRightDriveMotor: WrappedMotor,
      frontLeftTurnMotor: WrappedMotor,
      frontRightTurnMotor: WrappedMotor,
      backLeftTurnMotor: WrappedMotor,
      backRightTurnMotor: WrappedMotor,
      frontLeftLocation: Translation2d,
      driveMotorController: () -> PIDController,
      turnMotorController: () -> PIDController,
      driveFeedforward: SimpleMotorFeedforward,
      turnFeedforward: SimpleMotorFeedforward
    ): SwerveDrive {
      val modules = listOf(
        SwerveModule.create(
          "FLModule",
          frontLeftDriveMotor,
          frontLeftTurnMotor,
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          turnFeedforward,
          frontLeftLocation
        ),
        SwerveModule.create(
          "FRModule",
          frontRightDriveMotor,
          frontRightTurnMotor,
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          turnFeedforward,
          frontLeftLocation.rotateBy(Rotation2d.fromDegrees(-90.0))
        ),
        SwerveModule.create(
          "BLModule",
          backLeftDriveMotor,
          backLeftTurnMotor,
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          turnFeedforward,
          frontLeftLocation.rotateBy(Rotation2d.fromDegrees(90.0))
        ),
        SwerveModule.create(
          "BRModule",
          backRightDriveMotor,
          backRightTurnMotor,
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          turnFeedforward,
          frontLeftLocation.rotateBy(Rotation2d.fromDegrees(180.0))
        )
      )
      return SwerveDrive(
        modules,
        ahrs,
        maxLinearSpeed,
        maxRotSpeed
      )
    }

    /**
     * @return the sim version of this drive
     */
    fun simOf(swerve: SwerveDrive): SwerveSim {
      return SwerveSim(swerve.modules, swerve.ahrs, swerve.maxLinearSpeed, swerve.maxRotSpeed)
    }
  }
}
