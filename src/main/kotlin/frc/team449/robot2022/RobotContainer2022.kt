package frc.team449.robot2022

import edu.wpi.first.networktables.NetworkTableInstance
import frc.team449.RobotContainerBase
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax

class RobotContainer2022() : RobotContainerBase() {

  var autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed")
  var telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry")
  var rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate")

  var data = ""

  var counter = 0
  var startTime = 0.0
  var priorAutospeed = 0.0

  var numberArray = DoubleArray(10)
  var entries = ArrayList<Double>()

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
        DriveConstants.TURN_GEARING
      )
    )

  val motor = makeTurningMotor(
    "FL",
    DriveConstants.TURN_MOTOR_FL,
    false,
    DriveConstants.TURN_ENC_CHAN_FL,
    0.0
  )
  override fun teleopInit() {
    // todo Add button bindings here
  }

  override fun robotPeriodic() {
  }

  override fun simulationInit() {
  }

  override fun simulationPeriodic() {
  }
}
