package frc.team449.robot2022

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team449.RobotContainerBase
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax
import kotlin.math.PI

class RobotContainer2022() : RobotContainerBase() {

  private val joystick = XboxController(0)
  val PDP_CAN = 1
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
      enableBrakeMode = true,
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
//  val MODULE_FL = SwerveModule(
//    "FL",
//    DRIVE_FL,
//    TURN_FL,
//    PIDController(.2,.0,.0), /** DRIVE */
//    PIDController(1.0, .00,.0), /** TURN */
//    SimpleMotorFeedforward(DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV, DriveConstants.DRIVE_KA),
//    SimpleMotorFeedforward(DriveConstants.TURN_KS, DriveConstants.TURN_KV, DriveConstants.TURN_KA),
//    DriveConstants.FRONT_LEFT_LOC
//  )
  var control = PIDController(.01166, .021, .00021) // Zeigler Nicols Tuned

  private fun bindAngleToButton(angle: Double, buttonNumber: Int) {
    JoystickButton(joystick, buttonNumber).whenPressed(
      InstantCommand({
        control.setpoint = angle * PI / 180
      })
    )
  }

  override fun teleopInit() {
    control.enableContinuousInput(0.0, 2 * PI)
    control.setTolerance(1.0) // -_- useless
    bindAngleToButton(270.0, XboxController.Button.kA.value)
    bindAngleToButton(180.0, XboxController.Button.kX.value)
    bindAngleToButton(90.0, XboxController.Button.kY.value)
    bindAngleToButton(360.0, XboxController.Button.kB.value)
  }

  override fun teleopPeriodic() {
    val pid = control.calculate(TURN_FL.position)
    TURN_FL.set(pid)
  }
}
