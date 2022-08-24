package frc.team449.robot2022

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.team449.RobotContainerBase
import frc.team449.control.auto.AutoRoutine
import frc.team449.control.holonomic.OIHolonomic
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2022.auto.Example
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.system.AHRS
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.annotations.Log
import kotlin.math.abs

class RobotContainer2022() : RobotContainerBase() {

  // Other CAN IDs
  val PDP_CAN = 1
  val PCM_MODULE = 0

  private val driveController = XboxController(0)

  private val ahrs = AHRS(SerialPort.Port.kMXP)

  // Instantiate/declare PDP and other stuff here

  override val powerDistribution: PowerDistribution = PowerDistribution(PDP_CAN, PowerDistribution.ModuleType.kCTRE)

  /**
   * Converts the drive to a SwerveSim if the robot is in simulation
   */
  @Log.Include
  override val drive = if (isReal()) createDrivetrain() else SwerveDrive.simOf(createDrivetrain())

  override val autoChooser = addRoutines()

  override val oi = OIHolonomic(
    drive,
    { if (abs(driveController.leftY) < .08) .0 else driveController.leftY },
    { if (abs(driveController.leftX) < .08) .0 else driveController.leftX },
    { if (abs(driveController.getRawAxis(4)) < .02) .0 else driveController.getRawAxis(4) },
    SlewRateLimiter(1.5),
    2.5,
    true
  )

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
        DriveConstants.DRIVE_UPR,
        DriveConstants.DRIVE_GEARING
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
      enableBrakeMode = true,
      inverted = inverted,
      encCreator = AbsoluteEncoder.creator(
        encoderChannel,
        offset,
        DriveConstants.TURN_UPR,
        sensorPhase
      )
    )

  private fun createDrivetrain() =
    SwerveDrive.squareDrive(
      ahrs,
      DriveConstants.MAX_LINEAR_SPEED,
      DriveConstants.MAX_ROT_SPEED,
      makeDrivingMotor(
        "FL",
        DriveConstants.DRIVE_MOTOR_FL,
        false
      ),
      makeDrivingMotor(
        "FR",
        DriveConstants.DRIVE_MOTOR_FR,
        false
      ),
      makeDrivingMotor(
        "BL",
        DriveConstants.DRIVE_MOTOR_BL,
        false
      ),
      makeDrivingMotor(
        "BR",
        DriveConstants.DRIVE_MOTOR_BR,
        false
      ),
      makeTurningMotor(
        "FL",
        DriveConstants.TURN_MOTOR_FL,
        true,
        false,
        DriveConstants.TURN_ENC_CHAN_FL,
        DriveConstants.TURN_ENC_OFFSET_FL
      ),
      makeTurningMotor(
        "FR",
        DriveConstants.TURN_MOTOR_FR,
        true,
        false,
        DriveConstants.TURN_ENC_CHAN_FR,
        DriveConstants.TURN_ENC_OFFSET_FR
      ),
      makeTurningMotor(
        "BL",
        DriveConstants.TURN_MOTOR_BL,
        true,
        false,
        DriveConstants.TURN_ENC_CHAN_BL,
        DriveConstants.TURN_ENC_OFFSET_BL
      ),
      makeTurningMotor(
        "BR",
        DriveConstants.TURN_MOTOR_BR,
        true,
        false,
        DriveConstants.TURN_ENC_CHAN_BR,
        DriveConstants.TURN_ENC_OFFSET_BR
      ),
      DriveConstants.FRONT_LEFT_LOC,
      { PIDController(DriveConstants.DRIVE_KP, DriveConstants.DRIVE_KI, DriveConstants.DRIVE_KD) },
      {
        PIDController(DriveConstants.TURN_KP, DriveConstants.TURN_KI, DriveConstants.TURN_KD)
      },
      SimpleMotorFeedforward(DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV, DriveConstants.DRIVE_KA),
      SimpleMotorFeedforward(DriveConstants.TURN_KS, DriveConstants.TURN_KV, DriveConstants.TURN_KA)
    )

  private fun addRoutines(): SendableChooser<AutoRoutine> {
    val chooser = SendableChooser<AutoRoutine>()
    val exampleAuto = Example("Example", 2.0, 2.0, drive)
    chooser.setDefaultOption("Example Swerve Auto", exampleAuto.routine())

    return chooser
  }

  override fun teleopInit() {
    // todo Add button bindings here
  }

  override fun robotPeriodic() {
  }

  override fun simulationInit() {
    // DriverStationSim.setEnabled(true)
  }

  override fun simulationPeriodic() {
    // Update simulated mechanisms on Mechanism2d widget and stuff
  }
}
