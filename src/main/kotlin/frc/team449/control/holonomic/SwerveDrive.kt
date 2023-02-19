package frc.team449.control.holonomic

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.drives.SwerveConstants
import frc.team449.robot2023.constants.vision.VisionConstants
import frc.team449.system.AHRS
import frc.team449.system.VisionCamera
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.annotations.Log

/**
 * @param modules the list of swerve modules on this drivetrain
 * @param ahrs the gyro that is mounted on the chassis
 * @param maxLinearSpeed the maximum translation speed of the chassis.
 * @param maxRotSpeed the maximum rotation speed of the chassis
 */
open class SwerveDrive(
  private val modules: List<SwerveModule>,
  private val ahrs: AHRS,
  override val maxLinearSpeed: Double,
  override val maxRotSpeed: Double,
  private val cameras: List<VisionCamera> = mutableListOf()
) : SubsystemBase(), HolonomicDrive {

  private val kinematics = SwerveDriveKinematics(
    *this.modules
      .map { it.location }.toTypedArray()
  )

  @Log.ToString
  private var camPose = Pose2d()

  private var currentSpeeds = ChassisSpeeds()

  private val poseEstimator = SwerveDrivePoseEstimator(
    kinematics,
    ahrs.heading,
    getPositions(),
    RobotConstants.INITIAL_POSE,
    MatBuilder(Nat.N3(), Nat.N1()).fill(.005, .005, .005), // [theta, fl_pos, fr_pos, bl_pos, br_pos]
    MatBuilder(Nat.N3(), Nat.N1()).fill(.05, .075, .025) // [x, y, theta]
  )

  private var lastTime = Timer.getFPGATimestamp()

  @Log.ToString(name = "Desired Speeds")
  var desiredSpeeds = ChassisSpeeds()

  override fun set(desiredSpeeds: ChassisSpeeds) {
    this.desiredSpeeds = desiredSpeeds
  }

  /** The x y theta location of the robot on the field */
  override var pose: Pose2d
    @Log.ToString(name = "Pose")
    get() {
      return this.poseEstimator.estimatedPosition
    }
    set(value) {
      this.poseEstimator.resetPosition(
        ahrs.heading,
        getPositions(),
        value
      )
    }

  override fun stop() {
    this.set(ChassisSpeeds(0.0, 0.0, 0.0))
  }

  override fun periodic() {
    val currTime = Timer.getFPGATimestamp()

    currentSpeeds = kinematics.toChassisSpeeds(
      modules[0].state,
      modules[1].state,
      modules[2].state,
      modules[3].state
    )

    val desiredModuleStates =
      this.kinematics.toSwerveModuleStates(this.desiredSpeeds)

    /** If any module is going faster than the max speed,
     *  apply scaling down and make sure there isn't
     *  any early desaturation */
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredModuleStates,
      SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED
    )

    for (i in this.modules.indices) {
      this.modules[i].state = desiredModuleStates[i]
    }

    for (module in modules)
      module.update()

    if (cameras.isNotEmpty()) localize()

    this.poseEstimator.update(
      ahrs.heading,
      getPositions()
    )

    this.lastTime = currTime
  }

  /**
   * @return an array of [SwerveModulePosition] for each module, containing [angle, position]
   */
  private fun getPositions(): Array<SwerveModulePosition> {
    return Array(modules.size) { i -> modules[i].position }
  }

  /**
   * @return an array of [SwerveModuleState] for each module, containing [angle, velocity]
   */
  private fun getStates(): Array<SwerveModuleState> {
    return Array(modules.size) { i -> modules[i].state }
  }

  private fun localize() {
    for (camera in cameras) {
      if (camera.hasTarget()) {
        camPose = camera.camPose().toPose2d()
        poseEstimator.addVisionMeasurement(
          camPose,
          camera.timestamp()
        )
      }
    }
  }

  companion object {
    /** Create a swerve drivetrain using DriveConstants */
    fun swerveDrive(ahrs: AHRS): SwerveDrive {
      val driveMotorController = { PIDController(SwerveConstants.DRIVE_KP, SwerveConstants.DRIVE_KI, SwerveConstants.DRIVE_KD) }
      val turnMotorController = { PIDController(SwerveConstants.TURN_KP, SwerveConstants.TURN_KI, SwerveConstants.TURN_KD) }
      val driveFeedforward = SimpleMotorFeedforward(SwerveConstants.DRIVE_KS, SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA)
      val modules = listOf(
        SwerveModule.create(
          "FLModule",
          makeDrivingMotor(
            "FL",
            SwerveConstants.DRIVE_MOTOR_FL,
            inverted = false
          ),
          makeTurningMotor(
            "FL",
            SwerveConstants.TURN_MOTOR_FL,
            inverted = true,
            sensorPhase = false,
            SwerveConstants.TURN_ENC_CHAN_FL,
            SwerveConstants.TURN_ENC_OFFSET_FL
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(SwerveConstants.WHEELBASE / 2, SwerveConstants.TRACKWIDTH / 2)
        ),
        SwerveModule.create(
          "FRModule",
          makeDrivingMotor(
            "FR",
            SwerveConstants.DRIVE_MOTOR_FR,
            inverted = false
          ),
          makeTurningMotor(
            "FR",
            SwerveConstants.TURN_MOTOR_FR,
            inverted = true,
            sensorPhase = false,
            SwerveConstants.TURN_ENC_CHAN_FR,
            SwerveConstants.TURN_ENC_OFFSET_FR
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(SwerveConstants.WHEELBASE / 2, -SwerveConstants.TRACKWIDTH / 2)
        ),
        SwerveModule.create(
          "BLModule",
          makeDrivingMotor(
            "BL",
            SwerveConstants.DRIVE_MOTOR_BL,
            inverted = false
          ),
          makeTurningMotor(
            "BL",
            SwerveConstants.TURN_MOTOR_BL,
            inverted = true,
            sensorPhase = false,
            SwerveConstants.TURN_ENC_CHAN_BL,
            SwerveConstants.TURN_ENC_OFFSET_BL
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(-SwerveConstants.WHEELBASE / 2, SwerveConstants.TRACKWIDTH / 2)
        ),
        SwerveModule.create(
          "BRModule",
          makeDrivingMotor(
            "BR",
            SwerveConstants.DRIVE_MOTOR_BR,
            inverted = false
          ),
          makeTurningMotor(
            "BR",
            SwerveConstants.TURN_MOTOR_BR,
            inverted = true,
            sensorPhase = false,
            SwerveConstants.TURN_ENC_CHAN_BR,
            SwerveConstants.TURN_ENC_OFFSET_BR
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(-SwerveConstants.WHEELBASE / 2, -SwerveConstants.TRACKWIDTH / 2)
        )
      )
      return SwerveDrive(
        modules,
        ahrs,
        RobotConstants.MAX_LINEAR_SPEED,
        RobotConstants.MAX_ROT_SPEED,
        VisionConstants.CAMERAS
      )
    }

    /** Helper to make turning motors for swerve */
    private fun makeDrivingMotor(
      name: String,
      motorId: Int,
      inverted: Boolean
    ) =
      createSparkMax(
        name = name + "Drive",
        id = motorId,
        enableBrakeMode = true,
        inverted = inverted,
        encCreator =
        NEOEncoder.creator(
          SwerveConstants.DRIVE_UPR,
          SwerveConstants.DRIVE_GEARING
        )
      )

    /** Helper to make turning motors for swerve */
    private fun makeTurningMotor(
      name: String,
      motorId: Int,
      inverted: Boolean,
      sensorPhase: Boolean,
      encoderChannel: Int,
      offset: Double
    ) =
      createSparkMax(
        name = name + "Turn",
        id = motorId,
        enableBrakeMode = false,
        inverted = inverted,
        encCreator = AbsoluteEncoder.creator(
          encoderChannel,
          offset,
          SwerveConstants.TURN_UPR,
          sensorPhase
        )
      )
  }
}
