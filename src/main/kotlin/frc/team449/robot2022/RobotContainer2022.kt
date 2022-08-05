package frc.team449.robot2022

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import frc.team449.RobotContainerBase
import frc.team449.control.differential.DifferentialDrive
import frc.team449.control.differential.DifferentialOIs
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.system.AHRS
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.BackupEncoder
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.encoder.QuadEncoder
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.annotations.Log
import kotlin.math.absoluteValue

class RobotContainer2022 : RobotContainerBase() {

  // Other CAN IDs
  val PDP_CAN = 1

  val ahrs = AHRS(SerialPort.Port.kMXP)

  override val powerDistribution = PowerDistribution(PDP_CAN, PowerDistribution.ModuleType.kCTRE)

  // Instantiate/declare PDP and other stuff here

  val pdp = PowerDistribution()
  val calc = frc.team449.ResistanceCalculator()

  @Log.Include
  override val drive = createDrivetrain()

  override val driveSim = if (RobotBase.isSimulation()) createDriveSimController() else null

  val driveController = XboxController(DriveConstants.DRIVE_CONTROLLER_PORT)

  override val oi =
    DifferentialOIs.createCurvature(
      drive,
      { driveController.rightTriggerAxis - driveController.leftTriggerAxis },
      {
        if (driveController.leftX.absoluteValue < DriveConstants.DRIVE_TURNING_DEADBAND) .0
        else driveController.leftX
      },
      SlewRateLimiter(DriveConstants.LINEAR_ACC_LIMIT),
      SlewRateLimiter(DriveConstants.TURNING_ACC_LIMIT),
      { true }
    )

  /** Helper to make each side for the differential drive */
  private fun makeSide(
    name: String,
    motorId: Int,
    inverted: Boolean,
    wpiEnc: Encoder,
    followers: Map<Int, Boolean>
  ) =
    createSparkMax(
      name = name + "_Drive",
      id = motorId,
      enableBrakeMode = true,
      inverted = inverted,
      encCreator =
      BackupEncoder.creator(
        QuadEncoder.creator(
          wpiEnc,
          DriveConstants.DRIVE_EXT_ENC_CPR,
          DriveConstants.DRIVE_UPR,
          1.0
        ),
        NEOEncoder.creator(DriveConstants.DRIVE_UPR, DriveConstants.DRIVE_GEARING),
        DriveConstants.DRIVE_ENC_VEL_THRESHOLD
      ),
      slaveSparks = followers,
      currentLimit = DriveConstants.DRIVE_CURRENT_LIM
    )

  private fun createDrivetrain() =
    DifferentialDrive(
      leftLeader = makeSide(
        "Left",
        DriveConstants.DRIVE_MOTOR_L,
        false,
        DriveConstants.DRIVE_ENC_LEFT,
        mapOf(
          DriveConstants.DRIVE_MOTOR_L1 to false,
          DriveConstants.DRIVE_MOTOR_L2 to false
        )
      ),
      rightLeader = makeSide(
        "Right",
        DriveConstants.DRIVE_MOTOR_R,
        true,
        DriveConstants.DRIVE_ENC_RIGHT,
        mapOf(
          DriveConstants.DRIVE_MOTOR_R1 to false,
          DriveConstants.DRIVE_MOTOR_R2 to false
        )
      ),
      ahrs,
      SimpleMotorFeedforward(
        DriveConstants.DRIVE_FF_KS,
        DriveConstants.DRIVE_FF_KV,
        DriveConstants.DRIVE_FF_KA
      ),
      {
        PIDController(
          DriveConstants.DRIVE_KP_VEL,
          DriveConstants.DRIVE_KI_VEL,
          DriveConstants.DRIVE_KD_VEL
        )
      },
      DriveConstants.TRACK_WIDTH,
      DriveConstants.MAX_LINEAR_SPEED
    )

  private fun createDriveSimController() =
    DifferentialDrive.SimController(
      drive,
      DifferentialDrivetrainSim(
        LinearSystemId.identifyDrivetrainSystem(
          DriveConstants.DRIVE_FF_KV,
          DriveConstants.DRIVE_FF_KA,
          DriveConstants.DRIVE_ANGLE_FF_KV,
          DriveConstants.DRIVE_ANGLE_FF_KA
        ),
        DCMotor.getNEO(3),
        DriveConstants.DRIVE_GEARING,
        DriveConstants.TRACK_WIDTH,
        DriveConstants.DRIVE_WHEEL_RADIUS,
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
      ),
      AHRS.SimController()
    )
  var encoder = DutyCycleEncoder(3)
  var encoderObject = AbsoluteEncoder("coolEncoder", encoder, 2 * 3.14159, 2 * 3.14159, 1.0, false)
  override fun robotPeriodic() {
    super.robotPeriodic()
    println("Distance: ${encoderObject.position}")
    // println("Rotation angles : ${encoder.absolutePosition}")
  }
}
