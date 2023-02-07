package frc.team449.robot2023

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.XboxController
import frc.team449.RobotBase
import frc.team449.control.differential.DifferentialDrive.Companion.createDifferentialDrive
import frc.team449.control.differential.DifferentialOIs
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.subsystems.GroundIntake
import frc.team449.system.AHRS
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.annotations.Log

class Robot : RobotBase() {

  val driveController = XboxController(0)

  private val ahrs = AHRS(SerialPort.Port.kMXP)

  // Instantiate/declare PDP and other stuff here

  override val powerDistribution: PowerDistribution = PowerDistribution(RobotConstants.PDP_CAN, PowerDistribution.ModuleType.kCTRE)

  override val drive = createDifferentialDrive(ahrs)

  @Log(name = "Joystick Input")
  override val oi = DifferentialOIs.createCurvature(
    drive,
    { driveController.rightTriggerAxis - driveController.leftTriggerAxis },
    { driveController.leftX },
    SlewRateLimiter(RobotConstants.MAX_ACCEL),
    SlewRateLimiter(RobotConstants.RATE_LIMIT),
    { true }
  )

  override val intake = GroundIntake(
    createSparkMax("leftIntake", 6, NEOEncoder.creator(1.0, 1.0)),
    createSparkMax("leftIntake", 6, NEOEncoder.creator(1.0, 1.0), inverted = true),
  )
}
