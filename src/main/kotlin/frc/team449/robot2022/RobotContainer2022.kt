package frc.team449.robot2022

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.XboxController
import frc.team449.RobotContainerBase
import frc.team449.control.holonomic.SwerveModule
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax

class RobotContainer2022() : RobotContainerBase() {

  private val joystick = XboxController(0)

  val powerDistribution: PowerDistribution = PowerDistribution(PDP_CAN, PowerDistribution.ModuleType.kCTRE)

  /** Helper to make turning motors for swerve */
  private fun makeTurningMotor(
    name: String,
    motorId: Int,
    inverted: Boolean,
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
        DriveConstants.TURN_UPR
      )
    )
  private fun makeDrivingMotor(
    name: String,
    motorId: Int,
    inverted: Boolean
  ) = createSparkMax(
    name = name + "Drive",
    id = motorId,
    enableBrakeMode = true,
    inverted = inverted,
    encCreator = NEOEncoder.creator(
      DriveConstants.DRIVE_UPR,
      DriveConstants.DRIVE_GEARING
    )
  )
  val TURN_FL = makeTurningMotor(
    "FL",
    DriveConstants.TURN_MOTOR_FL,
    false,
    DriveConstants.TURN_ENC_CHAN_FL,
    DriveConstants.TURN_ENC_OFFSET_FL
  )
  val DRIVE_FL = makeDrivingMotor(
    "FL",
    DriveConstants.DRIVE_MOTOR_FL,
    false
  )

  val FL_MODULE = SwerveModule(
    "FL_MODULE",
    DRIVE_FL,
    TURN_FL,
    PIDController(.0, .0, .0),
    ProfiledPIDController(
      .0, .0, .0,
      TrapezoidProfile.Constraints(.1, .1)
    ),
    SimpleMotorFeedforward(DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV, DriveConstants.DRIVE_KA),
    SimpleMotorFeedforward(DriveConstants.TURN_KS, DriveConstants.TURN_KV, DriveConstants.TURN_KA),
    DriveConstants.FRONT_LEFT_LOC
  )
  override fun teleopInit() {
    // BIND Buttons
  }
}
