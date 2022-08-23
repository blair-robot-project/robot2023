package frc.team449.robot2022

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.XboxController
import frc.team449.RobotContainerBase
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax

class RobotContainer2022() : RobotContainerBase() {

  private val joystick = XboxController(0)
  val PDP_CAN = 1
  val powerDistribution: PowerDistribution = PowerDistribution(PDP_CAN, PowerDistribution.ModuleType.kCTRE)

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
    true,
    false,
    DriveConstants.TURN_ENC_CHAN_FL,
    DriveConstants.TURN_ENC_OFFSET_FL
  )
  val TURN_FR = makeTurningMotor(
    "FR",
    DriveConstants.TURN_MOTOR_FR,
    true,
    false,
    DriveConstants.TURN_ENC_CHAN_FR,
    DriveConstants.TURN_ENC_OFFSET_FR
  )
  val TURN_BL = makeTurningMotor(
    "BL",
    DriveConstants.TURN_MOTOR_BL,
    true,
    false,
    DriveConstants.TURN_ENC_CHAN_BL,
    DriveConstants.TURN_ENC_OFFSET_BL
  )
  val TURN_BR = makeTurningMotor(
    "BR",
    DriveConstants.TURN_MOTOR_BR,
    true,
    false,
    DriveConstants.TURN_ENC_CHAN_BR,
    DriveConstants.TURN_ENC_OFFSET_BR
  )
  val DRIVE_FL = makeDrivingMotor(
    "FL",
    DriveConstants.DRIVE_MOTOR_FL,
    false
  )
  val DRIVE_FR = makeDrivingMotor(
    "FR",
    DriveConstants.DRIVE_MOTOR_FR,
    false
  )
  val DRIVE_BL = makeDrivingMotor(
    "BL",
    DriveConstants.DRIVE_MOTOR_BL,
    false
  )
  val DRIVE_BR = makeDrivingMotor(
    "BR",
    DriveConstants.DRIVE_MOTOR_BR,
    false
  )
  override fun teleopInit() {
    DRIVE_BL.set(.25 / 12)
    DRIVE_BR.set(.25 / 12)
    DRIVE_FR.set(.25 / 12)
    DRIVE_FL.set(.25 / 12)
  }

  override fun disabledInit() {
    DRIVE_BL.set(.0)
    DRIVE_BR.set(.0)
    DRIVE_FR.set(.0)
    DRIVE_FL.set(.0)
  }
  override fun teleopPeriodic() {
  }
}
