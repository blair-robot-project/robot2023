package frc.team449.robot2022

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.XboxController
import frc.team449.RobotBase
import frc.team449.control.holonomic.MecanumDrive
import frc.team449.control.holonomic.OIHolonomic.Companion.createHolonomicOI
import frc.team449.robot2022.constants.RobotConstants
// import frc.team449.control.holonomic.SwerveDrive.Companion.swerveDrive
import frc.team449.system.AHRS
import io.github.oblarg.oblog.annotations.Log

class Robot : RobotBase() {

  val driveController = XboxController(0)

  private val ahrs = AHRS(SerialPort.Port.kMXP)

  // Instantiate/declare PDP and other stuff here

  override val powerDistribution: PowerDistribution = PowerDistribution(RobotConstants.PDP_CAN, PowerDistribution.ModuleType.kCTRE)

  override val drive = MecanumDrive.createMecanum(ahrs)

  @Log(name = "Joystick Input")
  override val oi = createHolonomicOI(drive, driveController)
}
