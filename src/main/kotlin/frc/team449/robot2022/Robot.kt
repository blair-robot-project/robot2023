package frc.team449.robot2022

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.XboxController
import frc.team449.RobotBase
import frc.team449.control.holonomic.MecanumDrive.Companion.createMecanum
import frc.team449.control.holonomic.OIHolonomic.Companion.createHolonomicOI
import frc.team449.system.AHRS
import io.github.oblarg.oblog.annotations.Log

class Robot : RobotBase() {

  // Other CAN IDs
  private val PDP_CAN = 1
  private val PCM_MODULE = 0

  private val driveController = XboxController(0)

  private val ahrs = AHRS(SerialPort.Port.kMXP)

  // Instantiate/declare PDP and other stuff here

  override val powerDistribution: PowerDistribution = PowerDistribution(PDP_CAN, PowerDistribution.ModuleType.kCTRE)

  override val drive = createMecanum(ahrs)

  @Log(name = "Joystick Input")
  override val oi = createHolonomicOI(drive, driveController)
}
