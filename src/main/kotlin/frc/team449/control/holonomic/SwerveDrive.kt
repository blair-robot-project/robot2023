// package frc.team449.control.holonomic
//
// import edu.wpi.first.math.MatBuilder
// import edu.wpi.first.math.Nat
// import edu.wpi.first.math.controller.PIDController
// import edu.wpi.first.math.controller.SimpleMotorFeedforward
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
// import edu.wpi.first.math.geometry.Pose2d
// import edu.wpi.first.math.geometry.Translation2d
// import edu.wpi.first.math.kinematics.ChassisSpeeds
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics
// import edu.wpi.first.math.kinematics.SwerveModulePosition
// import edu.wpi.first.math.kinematics.SwerveModuleState
// import edu.wpi.first.wpilibj.Timer
// import edu.wpi.first.wpilibj2.command.SubsystemBase
// import frc.team449.robot2022.drive.DriveConstants
// import frc.team449.system.AHRS
// import frc.team449.system.VisionCamera
// import frc.team449.system.encoder.AbsoluteEncoder
// import frc.team449.system.encoder.NEOEncoder
// import frc.team449.system.motor.createSparkMax
// import io.github.oblarg.oblog.annotations.Log
//
// /**
// * @param modules the list of swerve modules on this drivetrain
// * @param ahrs the gyro that is mounted on the chassis
// * @param maxLinearSpeed the maximum translation speed of the chassis.
// * @param maxRotSpeed the maximum rotation speed of the chassis
// */
// open class SwerveDrive(
//  private val modules: List<SwerveModule>,
//  private val ahrs: AHRS,
//  override val maxLinearSpeed: Double,
//  override val maxRotSpeed: Double,
//  private val cameras: MutableList<VisionCamera> = mutableListOf()
// ) : SubsystemBase(), HolonomicDrive {
//
//  private val kinematics = SwerveDriveKinematics(
//    *this.modules
//      .map { it.location }.toTypedArray()
//  )
//
//  @Log.ToString
//  private var camPose = Pose2d()
//
//  private val poseEstimator = SwerveDrivePoseEstimator(
//    kinematics,
//    ahrs.heading,
//    getPositions(),
//    DriveConstants.INITAL_POSE,
//    MatBuilder(Nat.N3(), Nat.N1()).fill(.005, .005, .005), // [theta, fl_pos, fr_pos, bl_pos, br_pos]
//    MatBuilder(Nat.N3(), Nat.N1()).fill(.05, .075, .025) // [x, y, theta]
//  )
//
//  private var lastTime = Timer.getFPGATimestamp()
//
//  @Log.ToString(name = "Desired Speeds")
//  var desiredSpeeds = ChassisSpeeds()
//
//  override fun set(desiredSpeeds: ChassisSpeeds) {
//    this.desiredSpeeds = desiredSpeeds
//  }
//
//  /** The x y theta location of the robot on the field */
//  override var pose: Pose2d
//    @Log.ToString(name = "Pose")
//    get() {
//      return this.poseEstimator.estimatedPosition
//    }
//    set(value) {
//      this.poseEstimator.resetPosition(
//        ahrs.heading,
//        getPositions(),
//        value
//      )
//    }
//
//  override fun stop() {
//    this.set(ChassisSpeeds(0.0, 0.0, 0.0))
//  }
//
//  override fun periodic() {
//    val currTime = Timer.getFPGATimestamp()
//
//    val desiredModuleStates =
//      this.kinematics.toSwerveModuleStates(this.desiredSpeeds)
//
//    /** If any module is going faster than the max speed,
//     *  apply scaling down */
//    SwerveDriveKinematics.desaturateWheelSpeeds(
//      desiredModuleStates,
//      DriveConstants.MAX_ATTAINABLE_MK4I_SPEED
//    )
//
//    for (i in this.modules.indices) {
//      this.modules[i].state = desiredModuleStates[i]
//    }
//
//    for (module in modules)
//      module.update()
//
//    if (cameras.isNotEmpty()) localize()
//
//    this.poseEstimator.update(
//      ahrs.heading,
//      getPositions()
//    )
//
//    this.lastTime = currTime
//  }
//
//  /**
//   * @return an array of [SwerveModulePosition] for each module, containing [angle, position]
//   */
//  private fun getPositions(): Array<SwerveModulePosition> {
//    return Array(modules.size) { i -> modules[i].position }
//  }
//
//  /**
//   * @return an array of [SwerveModuleState] for each module, containing [angle, velocity]
//   */
//  private fun getStates(): Array<SwerveModuleState> {
//    return Array(modules.size) { i -> modules[i].state }
//  }
//
//  private fun localize() {
//    for (camera in cameras) {
//      if (camera.hasTarget()) {
//        camPose = camera.camPose().toPose2d()
//        poseEstimator.addVisionMeasurement(
//          camPose,
//          camera.timestamp()
//        )
//      }
//    }
//  }
//
//  companion object {
//    /** Create a swerve drivetrain using DriveConstants */
// //    fun swerveDrive(ahrs: AHRS): SwerveDrive {
// //      val driveMotorController = { PIDController(DriveConstants.DRIVE_KP, DriveConstants.DRIVE_KI, DriveConstants.DRIVE_KD) }
// //      val turnMotorController = { PIDController(DriveConstants.TURN_KP, DriveConstants.TURN_KI, DriveConstants.TURN_KD) }
// //      val driveFeedforward = SimpleMotorFeedforward(DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV, DriveConstants.DRIVE_KA)
// //      val modules = listOf(
// //        SwerveModule.create(
// //          "FLModule",
// //          makeDrivingMotor(
// //            "FL",
// //            DriveConstants.DRIVE_MOTOR_FL,
// //            inverted = false
// //          ),
// //          makeTurningMotor(
// //            "FL",
// //            DriveConstants.TURN_MOTOR_FL,
// //            inverted = true,
// //            sensorPhase = false,
// //            DriveConstants.TURN_ENC_CHAN_FL,
// //            DriveConstants.TURN_ENC_OFFSET_FL
// //          ),
// //          driveMotorController(),
// //          turnMotorController(),
// //          driveFeedforward,
// //          Translation2d(DriveConstants.WHEELBASE / 2, DriveConstants.TRACKWIDTH / 2)
// //        ),
// //        SwerveModule.create(
// //          "FRModule",
// //          makeDrivingMotor(
// //            "FR",
// //            DriveConstants.DRIVE_MOTOR_FR,
// //            inverted = false
// //          ),
// //          makeTurningMotor(
// //            "FR",
// //            DriveConstants.TURN_MOTOR_FR,
// //            inverted = true,
// //            sensorPhase = false,
// //            DriveConstants.TURN_ENC_CHAN_FR,
// //            DriveConstants.TURN_ENC_OFFSET_FR
// //          ),
// //          driveMotorController(),
// //          turnMotorController(),
// //          driveFeedforward,
// //          Translation2d(DriveConstants.WHEELBASE / 2, - DriveConstants.TRACKWIDTH / 2)
// //        ),
// //        SwerveModule.create(
// //          "BLModule",
// //          makeDrivingMotor(
// //            "BL",
// //            DriveConstants.DRIVE_MOTOR_BL,
// //            inverted = false
// //          ),
// //          makeTurningMotor(
// //            "BL",
// //            DriveConstants.TURN_MOTOR_BL,
// //            inverted = true,
// //            sensorPhase = false,
// //            DriveConstants.TURN_ENC_CHAN_BL,
// //            DriveConstants.TURN_ENC_OFFSET_BL
// //          ),
// //          driveMotorController(),
// //          turnMotorController(),
// //          driveFeedforward,
// //          Translation2d(- DriveConstants.WHEELBASE / 2, DriveConstants.TRACKWIDTH / 2)
// //        ),
// //        SwerveModule.create(
// //          "BRModule",
// //          makeDrivingMotor(
// //            "BR",
// //            DriveConstants.DRIVE_MOTOR_BR,
// //            inverted = false
// //          ),
// //          makeTurningMotor(
// //            "BR",
// //            DriveConstants.TURN_MOTOR_BR,
// //            inverted = true,
// //            sensorPhase = false,
// //            DriveConstants.TURN_ENC_CHAN_BR,
// //            DriveConstants.TURN_ENC_OFFSET_BR
// //          ),
// //          driveMotorController(),
// //          turnMotorController(),
// //          driveFeedforward,
// //          Translation2d(- DriveConstants.WHEELBASE / 2, - DriveConstants.TRACKWIDTH / 2)
// //        )
// //      )
// //      return SwerveDrive(
// //        modules,
// //        ahrs,
// //        DriveConstants.MAX_LINEAR_SPEED,
// //        DriveConstants.MAX_ROT_SPEED,
// //        mutableListOf(VisionCamera(DriveConstants.CAM_NAME, DriveConstants.ROBOT_TO_CAM, DriveConstants.TAG_LAYOUT))
// //      )
// //    }
//
//    /** Helper to make turning motors for swerve */
//    private fun makeDrivingMotor(
//      name: String,
//      motorId: Int,
//      inverted: Boolean
//    ) =
//      createSparkMax(
//        name = name + "Drive",
//        id = motorId,
//        enableBrakeMode = true,
//        inverted = inverted,
//        encCreator =
//        NEOEncoder.creator(
//          DriveConstants.DRIVE_UPR,
//          DriveConstants.DRIVE_GEARING
//        )
//      )
//
//    /** Helper to make turning motors for swerve */
//    private fun makeTurningMotor(
//      name: String,
//      motorId: Int,
//      inverted: Boolean,
//      sensorPhase: Boolean,
//      encoderChannel: Int,
//      offset: Double
//    ) =
//      createSparkMax(
//        name = name + "Turn",
//        id = motorId,
//        enableBrakeMode = true,
//        inverted = inverted,
//        encCreator = AbsoluteEncoder.creator(
//          encoderChannel,
//          offset,
//          DriveConstants.TURN_UPR,
//          sensorPhase
//        )
//      )
//  }
// }
