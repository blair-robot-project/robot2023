package frc.team449.robot2022

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import frc.team449.RobotContainerBase
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.motor.createSparkMax

class RobotContainer2022() : RobotContainerBase() {

  var autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed")
  var telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry")

  var data = ""

  var counter = 0
  var startTime = 0.0
  var priorAutospeed = 0.0

  var numberArray = DoubleArray(10)
  var entries = ArrayList<Double>()

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

  val FL = makeTurningMotor(
    "FL",
    DriveConstants.TURN_MOTOR_FL,
    true,
    DriveConstants.TURN_ENC_CHAN_FL,
    DriveConstants.TURN_ENC_OFFSET_FL
  )
  override fun teleopInit() {
  }

  override fun disabledInit() {
    val elapsedTime = Timer.getFPGATimestamp() - startTime
    println("Robot disabled")
    FL.set(0.0)
    // data processing step
    // data processing step
    data = entries.toString()
    data = data.substring(1, data.length - 1) + ", "
    telemetryEntry.setString(data)
    entries.clear()
    println("Robot disabled")
    println("Collected : $counter in $elapsedTime seconds")
    data = ""
  }

  override fun robotPeriodic() {
  }

  override fun robotInit() {
    // Set the update rate instead of using flush because of a ntcore bug
    // -> probably don't want to do this on a robot in competition
//    NetworkTableInstance.getDefault().setUpdateRate(0.010)
  }
  override fun autonomousInit() {
//    super.autonomousInit()
    println("Robot in autonomous mode")
    startTime = Timer.getFPGATimestamp()
    counter = 0
  }

  override fun autonomousPeriodic() {

    // Retrieve values to send back before telling the motors to do something
    val now = Timer.getFPGATimestamp()

    val leftPosition: Double = FL.position
    val leftRate: Double = FL.velocity

    val rightPosition: Double = FL.position
    val rightRate: Double = FL.velocity

    val battery = RobotController.getBatteryVoltage()
    val motorVolts = battery * Math.abs(priorAutospeed)

    val leftMotorVolts = motorVolts
    val rightMotorVolts = motorVolts
    // Retrieve the commanded speed from NetworkTables

    // Retrieve the commanded speed from NetworkTables
    val autospeed = autoSpeedEntry.getDouble(0.0)
    priorAutospeed = autospeed

    // command motors to do things

    // command motors to do things
    FL.set(autospeed)

    numberArray[0] = now
    numberArray[1] = battery
    numberArray[2] = autospeed
    numberArray[3] = leftMotorVolts
    numberArray[4] = rightMotorVolts
    numberArray[5] = leftPosition
    numberArray[6] = rightPosition
    numberArray[7] = leftRate
    numberArray[8] = rightRate
    numberArray[9] = 0.0

    // Add data to a string that is uploaded to NT

    // Add data to a string that is uploaded to NT
    for (num in numberArray) {
      entries.add(num)
    }
    counter++
  }
}
